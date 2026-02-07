/* SPDX-License-Identifier: GPL-2.0-only
 *
 *      Low-level parallel-support for ax99100x pci/pcie to parallel port board
 *
 * 
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/pci.h>
#include <linux/pnp.h>
#include <linux/platform_device.h>
#include <linux/sysctl.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,11,0)	
#include <linux/sched.h>
#else
#include <linux/sched/signal.h>
#endif

#include <asm/dma.h>


#include <linux/via.h>

#if defined(__x86_64__) || defined(__amd64__)
// #include <asm/parport.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#endif
/****************************Parport Setting**********************************/
#define PARPORT_PCI_BASE	1 //0:iobase 1:membase
#define PARPORT_IRQ_ENABLE	0 //0:IRQ Disable 1:IRQ Enable

/****************************Parport Setting**********************************/

#if PARPORT_PCI_BASE
#include "parport.h"
#else
#include <linux/parport.h>
#include <linux/parport_pc.h>
#endif
#include "ax99100x_pp.h"




// #define PARPORT_PC_MAX_PORTS PARPORT_MAX

#ifdef CONFIG_ISA_DMA_API
#define HAS_DMA
#endif

/*copy register define from parport_ax88796.c*/
#define AX_SPR_BUSY		(1<<7)
#define AX_SPR_ACK		(1<<6)
#define AX_SPR_PE		(1<<5)
#define AX_SPR_SLCT		(1<<4)
#define AX_SPR_ERR		(1<<3)

#define AX_CPR_nDOE		(1<<5)
#define AX_CPR_SLCTIN		(1<<3)
#define AX_CPR_nINIT		(1<<2)
#define AX_CPR_ATFD		(1<<1)
#define AX_CPR_STRB		(1<<0)

/* ECR modes */
#define ECR_SPP 00
#define ECR_PS2 01
#define ECR_PPF 02
#define ECR_ECP 03
#define ECR_EPP 04
#define ECR_VND 05
#define ECR_TST 06
#define ECR_CNF 07
#define ECR_MODE_MASK 0xe0
#define ECR_WRITE(p, v) frob_econtrol((p), 0xff, (v))


/*
 * Definitions for PCI support.
 */
#define FL_BASE_MASK			0x0007
#define FL_BASE0			0x0000
#define FL_BASE1			0x0001
#define FL_BASE2			0x0002
#define FL_BASE3			0x0003
#define FL_BASE4			0x0004
#define FL_BASE5			0x0005
#define FL_GET_BASE(x)			(x & FL_BASE_MASK)

#define PARPORT_FIFO 1
#define CONFIG_PARPORT_1284 1




#if PARPORT_PCI_BASE
static int user_specified = 0; // for probing parport given by user in parport_pc.c, won't use in this driver.  
static int verbose_probing = 0;// for Log chit-chat during initialization, won't use by default.

#if LINUX_VERSION_CODE < KERNEL_VERSION(6,3,0)	
/* frob_control, but for ECR */
static void frob_econtrol(struct parport *pb, unsigned char m,
			   unsigned char v)
{
	struct pci_parport_data *priv = pb->private_data;
	unsigned char ectr = 0;

	if (m != 0xff)
		ectr = readb(priv->ectrl);//inb(ECONTROL(pb));

	DEBUG("frob_econtrol(%02x,%02x): %02x -> %02x\n",
		 m, v, ectr, (ectr & ~m) ^ v);

	writeb((ectr & ~m) ^ v, priv->ectrl);//outb((ectr & ~m) ^ v, ECONTROL(pb));
}
#else
/* frob_control, but for ECR */
static void frob_econtrol(struct parport *pb, unsigned char m,
			   unsigned char v)
{
	struct pci_parport_data *priv = pb->private_data;
	unsigned char ecr_writable = priv->ecr_writable;
	unsigned char ectr = 0;
	unsigned char new;

	// struct pci_parport_data *priv = pp_to_drv(p);	
	// return readb(priv->status); //inb (STATUS (p));

	if (m != 0xff)
		ectr = readb(priv->ectrl);//inb(ECONTROL(pb));

	new = (ectr & ~m) ^ v;
	if (ecr_writable)
		/* All known users of the ECR mask require bit 0 to be set. */
		new = (new & ecr_writable) | 1;

	DEBUG("frob_econtrol(%02x,%02x): %02x -> %02x\n", m, v, ectr, new);

	writeb(new, priv->ectrl);//outb(new, ECONTROL(pb));

}
#endif

static inline void frob_set_mode(struct parport *p, int mode)
{

	frob_econtrol(p, ECR_MODE_MASK, mode << 5);

}

/*
 * Clear TIMEOUT BIT in EPP MODE
 *
 * This is also used in SPP detection.
 */
static int clear_epp_timeout(struct parport *pb)
{
	unsigned char r;
	struct pci_parport_data *priv = pb->private_data;

	if (!(parport_ax_read_status(pb) & 0x01))
		return 1;

	/* To clear timeout some chips require double read */
	parport_ax_read_status(pb);
	r = parport_ax_read_status(pb);
	writeb(r | 0x01, priv->status); // outb(r | 0x01, STATUS(pb)); /* Some reset by writing 1 */
	writeb(r & 0xfe, priv->status); // outb(r & 0xfe, STATUS(pb)); /* Others by writing 0 */
	r = parport_ax_read_status(pb);

	return !(r & 0x01);
}

/*
 * Access functions.
 *
 * Most of these aren't static because they may be used by the
 * parport_xxx_yyy macros.  extern __inline__ versions of several
 * of these are in parport_pc.h.
 */

static void parport_ax_init_state(struct pardevice *dev,
						struct parport_state *s)
{

	s->u.pc.ctr = 0xc;
	if (dev->irq_func &&
	    dev->port->irq != PARPORT_IRQ_NONE)
		/* Set ackIntEn */
		s->u.pc.ctr |= 0x10;

	s->u.pc.ecr = 0x34; /* NetMos chip can cause problems 0x24;
			     * D.Gruszka VScom */

}

static void parport_ax_save_state(struct parport *p, struct parport_state *s)
{
	const struct pci_parport_data *priv = p->private_data;

	s->u.pc.ctr = priv->ctr;
	if (priv->ecr){
		s->u.pc.ecr = readb(priv->ectrl);//s->u.pc.ecr = inb(ECONTROL(p));

	}

}

static void parport_ax_restore_state(struct parport *p,
						struct parport_state *s)
{
	struct pci_parport_data *priv = p->private_data;
	register unsigned char c = s->u.pc.ctr & priv->ctr_writable;

	writeb(c, priv->ctrl);//outb(c, CONTROL(p));
	priv->ctr = c;
	if (priv->ecr) {
		ECR_WRITE(p, s->u.pc.ecr);

	}

}


static struct parport_operations parport_ax99100pp_ops = {
	.write_data	= parport_ax_write_data,
	.read_data	= parport_ax_read_data,

	.write_control	= parport_ax_write_control,
	.read_control	= parport_ax_read_control,
	.frob_control	= parport_ax_frob_control,

	.read_status	= parport_ax_read_status,

	.enable_irq	= parport_ax_enable_irq,
	.disable_irq	= parport_ax_disable_irq,

	.data_forward	= parport_ax_data_forward,
	.data_reverse	= parport_ax_data_reverse,

	.init_state	= parport_ax_init_state,
	.save_state	= parport_ax_save_state,
	.restore_state	= parport_ax_restore_state,

	.epp_write_data	= parport_ieee1284_epp_write_data,
	.epp_read_data	= parport_ieee1284_epp_read_data,
	.epp_write_addr	= parport_ieee1284_epp_write_addr,
	.epp_read_addr	= parport_ieee1284_epp_read_addr,

	.ecp_write_data	= parport_ieee1284_ecp_write_data,
	.ecp_read_data	= parport_ieee1284_ecp_read_data,
	.ecp_write_addr	= parport_ieee1284_ecp_write_addr,

	.compat_write_data	= parport_ieee1284_write_compat,
	.nibble_read_data	= parport_ieee1284_read_nibble,
	.byte_read_data		= parport_ieee1284_read_byte,

	.owner		= THIS_MODULE,
};




/* --- Mode detection ------------------------------------- */

/*
 * Checks for port existence, all ports support SPP MODE
 * Returns:
 *         0           :  No parallel port at this address
 *  PARPORT_MODE_PCSPP :  SPP port detected
 *                        (if the user specified an ioport himself,
 *                         this shall always be the case!)
 *
 */
static int parport_SPP_supported(struct parport *pb)
{
	unsigned char r, w;
	struct pci_parport_data *priv = pb->private_data;

	/*
	 * first clear an eventually pending EPP timeout
	 * I (sailer@ife.ee.ethz.ch) have an SMSC chipset
	 * that does not even respond to SPP cycles if an EPP
	 * timeout is pending
	 */
	clear_epp_timeout(pb);

	/* Do a simple read-write test to make sure the port exists. */
	w = 0xc;
	writeb(w, priv->ctrl);//outb(w, CONTROL(pb));

	/* Is there a control register that we can read from?  Some
	 * ports don't allow reads, so read_control just returns a
	 * software copy. Some ports _do_ allow reads, so bypass the
	 * software copy here.  In addition, some bits aren't
	 * writable. */
	r = readb(priv->ctrl);//r = inb(CONTROL(pb));

	if ((r & 0xf) == w) {

		w = 0xe;
		writeb(w, priv->ctrl);//outb(w, CONTROL(pb));
		r = readb(priv->ctrl);//r = inb(CONTROL(pb));
		writeb(0xc, priv->ctrl);//outb(0xc, CONTROL(pb));

		if ((r & 0xf) == w){

			return PARPORT_MODE_PCSPP;
		}
	}

	if (user_specified)
		/* That didn't work, but the user thinks there's a
		 * port here. */
		pr_info("parport 0x%lx (WARNING): CTR: wrote 0x%02x, read 0x%02x\n",
			pb->base, w, r);

	/* Try the data register.  The data lines aren't tri-stated at
	 * this stage, so we expect back what we wrote. */
	w = 0xaa;
	parport_ax_write_data(pb, w);
	r = parport_ax_read_data(pb);
	if (r == w) {
		w = 0x55;
		parport_ax_write_data(pb, w);
		r = parport_ax_read_data(pb);

		if (r == w){

			return PARPORT_MODE_PCSPP;
		}
	}


	if (user_specified) {
		/* Didn't work, but the user is convinced this is the
		 * place. */
		pr_info("parport 0x%lx (WARNING): DATA: wrote 0x%02x, read 0x%02x\n",
			pb->base, w, r);
		pr_info("parport 0x%lx: You gave this address, but there is probably no parallel port there!\n",
			pb->base);
	}


	/* It's possible that we can't read the control register or
	 * the data register.  In that case just believe the user. */
	if (user_specified)
		return PARPORT_MODE_PCSPP;

	return 0;
}

/* Check for ECR
 *
 * Old style XT ports alias io ports every 0x400, hence accessing ECR
 * on these cards actually accesses the CTR.
 *
 * Modern cards don't do this but reading from ECR will return 0xff
 * regardless of what is written here if the card does NOT support
 * ECP.
 *
 * We first check to see if ECR is the same as CTR.  If not, the low
 * two bits of ECR aren't writable, so we check by writing ECR and
 * reading it back to see if it's what we expect.
 */
static int parport_ECR_present(struct parport *pb)
{
	struct pci_parport_data *priv = pb->private_data;
	unsigned char r = 0xc;

	if (!priv->ecr_writable) {
		writeb(r, priv->ctrl);//outb(r, CONTROL(pb));

		// if ((inb(ECONTROL(pb)) & 0x3) == (r & 0x3)) {
		if ((readb(priv->ectrl) & 0x3) == (r & 0x3)) {
			// outb(r ^ 0x2, CONTROL(pb)); /* Toggle bit 1 */
			writeb(r ^ 0x2, priv->ctrl); /* Toggle bit 1 */


			r = readb(priv->ectrl);//r = inb(CONTROL(pb));
			
			// if ((inb(ECONTROL(pb)) & 0x2) == (r & 0x2))
			if ((readb(priv->ectrl) & 0x2) == (r & 0x2))
				/* Sure that no ECR register exists */
				goto no_reg;
		}

		// if ((inb(ECONTROL(pb)) & 0x3) != 0x1)
		if ((readb(priv->ectrl) & 0x3) != 0x1)
			goto no_reg;

		ECR_WRITE(pb, 0x34);
		// if (inb(ECONTROL(pb)) != 0x35)
		if (readb(priv->ectrl) != 0x35)
			goto no_reg;
	}

	priv->ecr = 1;
	writeb(0xc, priv->ctrl);//outb(0xc, CONTROL(pb));

	/* Go to mode 000 */
	frob_set_mode(pb, ECR_SPP);


	return 1;

 no_reg:

	writeb(0xc, priv->ctrl);//outb(0xc, CONTROL(pb));
	return 0;
}

/* Detect PS/2 support.
 *
 * Bit 5 (0x20) sets the PS/2 data direction; setting this high
 * allows us to read data from the data lines.  In theory we would get back
 * 0xff but any peripheral attached to the port may drag some or all of the
 * lines down to zero.  So if we get back anything that isn't the contents
 * of the data register we deem PS/2 support to be present.
 *
 * Some SPP ports have "half PS/2" ability - you can't turn off the line
 * drivers, but an external peripheral with sufficiently beefy drivers of
 * its own can overpower them and assert its own levels onto the bus, from
 * where they can then be read back as normal.  Ports with this property
 * and the right type of device attached are likely to fail the SPP test,
 * (as they will appear to have stuck bits) and so the fact that they might
 * be misdetected here is rather academic.
 */

static int parport_PS2_supported(struct parport *pb)
{
	struct pci_parport_data *priv = pb->private_data;
	int ok = 0;

	clear_epp_timeout(pb);

	/* try to tri-state the buffer */
	parport_ax_data_reverse(pb);

	parport_ax_write_data(pb, 0x55);

	if (parport_ax_read_data(pb) != 0x55){

		ok++;
	}
		

	parport_ax_write_data(pb, 0xaa);

	if (parport_ax_read_data(pb) != 0xaa){

		ok++;
	}

		

	/* cancel input mode */
	parport_ax_data_forward(pb);

	if (ok) {

		pb->modes |= PARPORT_MODE_TRISTATE;
	} else {

		// struct pci_parport_data *priv = pb->private_data;
		priv->ctr_writable &= ~0x20;
	}

	return ok;
}

static int parport_ECP_supported(struct parport *pb)
{
	int i;
	int config, configb;
	int pword;
	unsigned char gier;

	// struct parport_pc_private *priv = pb->private_data;
	struct pci_parport_data *priv = pb->private_data;

	/* Translate ECP intrLine to ISA irq value */
	static const int intrline[] = { 0, 7, 9, 10, 11, 14, 15, 5 };
	printk(KERN_DEBUG "gisr=0x%02x, gicr=0x%02x, gier=0x%02x\r\n", readb(priv->membase + 0x3a0), readb(priv->membase + 0x3a4), readb(priv->membase + 0x3a8));
	/* If there is no ECR, we have no hope of supporting ECP. */
	if (!priv->ecr)
	{
		return 0;
	}

	frob_econtrol(pb,1<<2,1<<2);// Clear and Disable service interrupt
	gier = readb(priv->membase + 0x3a8);//Backup GIER
	writeb(0x01, priv->membase + 0x3a4);//Clear service interrupt status
	writeb(gier & (~0x01), priv->membase + 0x3a8);//Disable service interrupt

	/* Find out FIFO depth */
	ECR_WRITE(pb, ECR_SPP << 5); /* Reset FIFO */
	ECR_WRITE(pb, ECR_TST << 5); /* TEST FIFO */

	for (i = 0; i < 1024 && !(readb(priv->ectrl) & 0x02); i++)
	{
		writeb(0xaa, priv->fifo);
	}

	/*
	 * Using LGS chipset it uses ECR register, but
	 * it doesn't support ECP or FIFO MODE
	 */
	if (i == 1024) {
		ECR_WRITE(pb, ECR_SPP << 5);

		writeb(0x01, priv->membase + 0x3a4);//Clear service interrupt status
		writeb(gier, priv->membase + 0x3a8);//Restore IGER

		return 0;
	}

	priv->fifo_depth = i;

	if (verbose_probing)
	{
		printk(KERN_DEBUG "0x%lx: FIFO is %d bytes\n", pb->base, i);
	}

	/* Find out writeIntrThreshold */
//	frob_econtrol(pb, 1<<2, 1<<2);// Clear and Disable service interrupt
//	frob_econtrol(pb, 1<<2, 0);// Enable service interrupt and reset Test mode
	for (i = 1; i <= priv->fifo_depth; i++) {
		readb(priv->fifo);
		udelay(50);
//			if (readb(priv->ectrl) & (1<<2)){	
			if (readb(priv->ectrl) & (1<<0)){
			break;
		}
			
	}

	if (i <= priv->fifo_depth) {
		if (verbose_probing)
		{
			printk(KERN_DEBUG "0x%lx: writeIntrThreshold is %d\n", pb->base, i);	
		}
	} else {
		/* Number of bytes we know we can write if we get an
		   interrupt. */
		i = readb(priv->membase + 0x258);
	}


	priv->writeIntrThreshold = i;

	/* Find out readIntrThreshold */
	frob_set_mode(pb, ECR_PS2); /* Reset FIFO and enable PS2 */
	parport_ax_data_reverse(pb); /* Must be in PS2 mode */
	frob_set_mode(pb, ECR_TST); /* Test FIFO */

//	frob_econtrol(pb, 1<<2, 1<<2);// Clear and Disable service interrupt
//	frob_econtrol(pb, 1<<2, 0);// Enable service interrupt
	for (i = 1; i <= priv->fifo_depth; i++) {
		writeb(0xaa,priv->fifo);

//		if (readb(priv->ectrl) & (1<<2))
//			break;
	}


	if (i <= priv->fifo_depth) {
		if (verbose_probing)
		{
			pr_info("0x%lx: readIntrThreshold is %d\n",	pb->base, i);		
		}
	} else {
		/* Number of bytes we can read if we get an interrupt. */
		i = readb(priv->membase + 0x254);
	}

	priv->readIntrThreshold = i;

	ECR_WRITE(pb, ECR_SPP << 5); /* Reset FIFO */
	ECR_WRITE(pb, 0xf4); /* Configuration mode */
	config = readb(priv->configa);
	pword = (config >> 4) & 0x7;
	switch (pword) {
	case 0:
		pword = 2;
		pr_warn("0x%lx: Unsupported pword size!\n", pb->base);
		break;
	case 2:
		pword = 4;
		pr_warn("0x%lx: Unsupported pword size!\n", pb->base);
		break;
	default:
		pr_warn("0x%lx: Unknown implementation ID\n", pb->base);
		//fallthrough;	/* Assume 1 */
	case 1:
		pword = 1;
	}
	priv->pword = pword;

	if (verbose_probing) {
		printk(KERN_DEBUG "0x%lx: PWord is %d bits\n", pb->base, 8 * pword);
		printk(KERN_DEBUG "0x%lx: Interrupts are ISA-%s\n", pb->base, config & 0x80 ? "Level" : "Pulses");

		configb = readb(priv->configb);
		printk(KERN_DEBUG "0x%lx: ECP port cfgA=0x%02x cfgB=0x%02x\n", pb->base, config, configb);
		printk(KERN_DEBUG "0x%lx: ECP settings irq=", pb->base);
		if ((configb >> 3) & 0x07)
			pr_cont("%d", intrline[(configb >> 3) & 0x07]);
		else
			pr_cont("<none or set by other means>");
		pr_cont(" dma=");
		if ((configb & 0x03) == 0x00)
			pr_cont("<none or set by other means>\n");
		else
			pr_cont("%d\n", configb & 0x07);
	}

	/* Go back to mode 000 */
	frob_set_mode(pb, ECR_SPP);

	printk(KERN_DEBUG "fifo_depth=0x%02x, writeIntrThreshold=0x%02x, readIntrThreshold=0x%02x\r\n", priv->fifo_depth, priv->writeIntrThreshold, priv->readIntrThreshold);

	frob_econtrol(pb, 1<<2, 1<<2);// Clear and Disable service interrupt
	writeb(0x01, priv->membase + 0x3a4);//Clear service interrupt status
	writeb(gier, priv->membase + 0x3a8);//Restore GIER
	return 1;
}

/* EPP mode detection  */

static int parport_EPP_supported(struct parport *pb)
{
	/*
	 * Theory:
	 *	Bit 0 of STR is the EPP timeout bit, this bit is 0
	 *	when EPP is possible and is set high when an EPP timeout
	 *	occurs (EPP uses the HALT line to stop the CPU while it does
	 *	the byte transfer, an EPP timeout occurs if the attached
	 *	device fails to respond after 10 micro seconds).
	 *
	 *	This bit is cleared by either reading it (National Semi)
	 *	or writing a 1 to the bit (SMC, UMC, WinBond), others ???
	 *	This bit is always high in non EPP modes.
	 */

	/* If EPP timeout bit clear then EPP available */
	if (!clear_epp_timeout(pb))
		return 0;  /* No way to clear timeout */

	pb->modes |= PARPORT_MODE_EPP;

	/* Set up access functions to use EPP hardware. */
	// pb->ops->epp_read_data = parport_ax_epp_read_data;
	// pb->ops->epp_write_data = parport_ax_epp_write_data;
	// pb->ops->epp_read_addr = parport_ax_epp_read_addr;
	// pb->ops->epp_write_addr = parport_ax_epp_write_addr;

	return 1;
}

static int parport_ECPEPP_supported(struct parport *pb)
{
	// struct parport_pc_private *priv = pb->private_data;
	struct pci_parport_data *priv = pb->private_data;
	int result;
	unsigned char oecr;

	if (!priv->ecr)
		return 0;

	oecr = readb(priv->ectrl);//oecr = inb(ECONTROL(pb));
	/* Search for SMC style EPP+ECP mode */
	ECR_WRITE(pb, 0x80);
	writeb(0x04, priv->ctrl);//outb(0x04, CONTROL(pb));
	result = parport_EPP_supported(pb);

	ECR_WRITE(pb, oecr);

	// if (result) {
	// 	/* Set up access functions to use ECP+EPP hardware. */
	// 	pb->ops->epp_read_data = parport_ax_ecpepp_read_data;
	// 	pb->ops->epp_write_data = parport_ax_ecpepp_write_data;
	// 	pb->ops->epp_read_addr = parport_ax_ecpepp_read_addr;
	// 	pb->ops->epp_write_addr = parport_ax_ecpepp_write_addr;
	// }

	return result;
}

static int parport_ECPPS2_supported(struct parport *pb)
{
	// const struct parport_pc_private *priv = pb->private_data;
	struct pci_parport_data *priv = pb->private_data;
	int result;
	unsigned char oecr;

	if (!priv->ecr)
		return 0;

	// oecr = inb(ECONTROL(pb));
	oecr = readb(priv->ectrl);
	ECR_WRITE(pb, ECR_PS2 << 5);
	result = parport_PS2_supported(pb);
	ECR_WRITE(pb, oecr);
	return result;
}



/* --- IRQ detection -------------------------------------- */

/* Only if supports ECP mode */
static int programmable_irq_support(struct parport *pb)
{
	struct pci_parport_data *priv = pb->private_data;
	int irq, intrLine;
	// unsigned char oecr = inb(ECONTROL(pb));
	unsigned char oecr = readb(priv->ectrl);
	static const int lookup[8] = {
		PARPORT_IRQ_NONE, 7, 9, 10, 11, 14, 15, 5
	};

	ECR_WRITE(pb, ECR_CNF << 5); /* Configuration MODE */

	// intrLine = (inb(CONFIGB(pb)) >> 3) & 0x07;
	intrLine = (readb(priv->configb) >> 3) & 0x07;
	irq = lookup[intrLine];

	ECR_WRITE(pb, oecr);
	return irq;
}

static int irq_probe_ECP(struct parport *pb)
{
	int i;
	unsigned long irqs;
	struct pci_parport_data *priv = pb->private_data;

	irqs = probe_irq_on();

	ECR_WRITE(pb, ECR_SPP << 5); /* Reset FIFO */
	ECR_WRITE(pb, (ECR_TST << 5) | 0x04);
	ECR_WRITE(pb, ECR_TST << 5);

	/* If Full FIFO sure that writeIntrThreshold is generated */
	// for (i = 0; i < 1024 && !(inb(ECONTROL(pb)) & 0x02) ; i++)
	// 	outb(0xaa, FIFO(pb));

	for (i = 0; i < 1024 && !(readb(priv->ectrl) & 0x02) ; i++)
		writeb(0xaa, priv->fifo);

	pb->irq = probe_irq_off(irqs);
	ECR_WRITE(pb, ECR_SPP << 5);

	if (pb->irq <= 0)
		pb->irq = PARPORT_IRQ_NONE;

	return pb->irq;
}

/*
 * This detection seems that only works in National Semiconductors
 * This doesn't work in SMC, LGS, and Winbond
 */
static int irq_probe_EPP(struct parport *pb)
{
#ifndef ADVANCED_DETECT
	return PARPORT_IRQ_NONE;
#else
	int irqs;
	unsigned char oecr;

	if (pb->modes & PARPORT_MODE_PCECR)
		oecr = inb(ECONTROL(pb));

	irqs = probe_irq_on();

	if (pb->modes & PARPORT_MODE_PCECR)
		frob_econtrol(pb, 0x10, 0x10);

	clear_epp_timeout(pb);
	parport_ax_frob_control(pb, 0x20, 0x20);
	parport_ax_frob_control(pb, 0x10, 0x10);
	clear_epp_timeout(pb);

	/* Device isn't expecting an EPP read
	 * and generates an IRQ.
	 */
	parport_ax_read_epp(pb);
	udelay(20);

	pb->irq = probe_irq_off(irqs);
	if (pb->modes & PARPORT_MODE_PCECR)
		ECR_WRITE(pb, oecr);
	parport_ax_write_control(pb, 0xc);

	if (pb->irq <= 0)
		pb->irq = PARPORT_IRQ_NONE;

	return pb->irq;
#endif /* Advanced detection */
}

// static int irq_probe_SPP(struct parport *pb)
// {
// 	/* Don't even try to do this. */
// 	return PARPORT_IRQ_NONE;
// }

/* We will attempt to share interrupt requests since other devices
 * such as sound cards and network cards seem to like using the
 * printer IRQs.
 *
 * When ECP is available we can autoprobe for IRQs.
 * NOTE: If we can autoprobe it, we can register the IRQ.
 */
static int parport_irq_probe(struct parport *pb)
{
	struct pci_parport_data *priv = pb->private_data;

	if (priv->ecr) {
		pb->irq = programmable_irq_support(pb);

		if (pb->irq == PARPORT_IRQ_NONE)
			pb->irq = irq_probe_ECP(pb);
	}

	if ((pb->irq == PARPORT_IRQ_NONE) && priv->ecr &&
	    (pb->modes & PARPORT_MODE_EPP))
		pb->irq = irq_probe_EPP(pb);

	clear_epp_timeout(pb);

	if (pb->irq == PARPORT_IRQ_NONE && (pb->modes & PARPORT_MODE_EPP))
		pb->irq = irq_probe_EPP(pb);

	clear_epp_timeout(pb);

	if (pb->irq == PARPORT_IRQ_NONE)
		;//pb->irq = irq_probe_SPP(pb);

	if (pb->irq == PARPORT_IRQ_NONE)
		;//pb->irq = get_superio_irq(pb);

	return pb->irq;
}

/* --- DMA detection -------------------------------------- */

/* Only if chipset conforms to ECP ISA Interface Standard */
static int programmable_dma_support(struct parport *p)
{
	struct pci_parport_data *priv = p->private_data;
	// unsigned char oecr = inb(ECONTROL(p));
	unsigned char oecr = readb(priv->ectrl);
	int dma;

	frob_set_mode(p, ECR_CNF);

	// dma = inb(CONFIGB(p)) & 0x07;
	dma = readb(priv->configb) & 0x07;
	/* 000: Indicates jumpered 8-bit DMA if read-only.
	   100: Indicates jumpered 16-bit DMA if read-only. */
	if ((dma & 0x03) == 0)
		dma = PARPORT_DMA_NONE;

	ECR_WRITE(p, oecr);
	return dma;
}

static int parport_dma_probe(struct parport *p)
{
	const struct pci_parport_data *priv = p->private_data;
	if (priv->ecr)		/* ask ECP chipset first */
		p->dma = programmable_dma_support(p);
	if (p->dma == PARPORT_DMA_NONE) {
		/* ask known Super-IO chips proper, although these
		   claim ECP compatible, some don't report their DMA
		   conforming to ECP standards */
		;//p->dma = get_superio_dma(p);
	}

	return p->dma;
}


/*------Init code functions---------------*/
static LIST_HEAD(ports_list);
static DEFINE_SPINLOCK(ports_lock);


void parport_ax_unregister_port(struct parport *p)
{
	struct pci_parport_data *priv = p->private_data;
	struct parport_operations *ops = p->ops;

	parport_remove_port(p);
	spin_lock(&ports_lock);
	list_del_init(&priv->list);
	spin_unlock(&ports_lock);
	if (p->dma != PARPORT_DMA_NONE)
		free_dma(p->dma);
	if (p->irq != PARPORT_IRQ_NONE)
		free_irq(p->irq, p);
	if (priv->dma_buf)
		dma_free_coherent(priv->dev, PAGE_SIZE,
				    priv->dma_buf,
				    priv->dma_handle);
	kfree(p->private_data);
	parport_del_port(p);
	kfree(ops); /* hope no-one cached it */
}

static int parport_ax_init_check(struct parport *p){
	struct pci_parport_data *priv = p->private_data;
	// struct parport_operations *ops = p->ops;
	int probedirq = PARPORT_IRQ_NONE;
	/* checked Linux source, ecr_writable always set to 0 in function "parport_pc_probe_port" in parport_pc.c */
	unsigned char ecr_writable=0;
	// unsigned int mode_mask=0;
	int ret, dma;
	p->name="ax99100x_pp";
	DEBUG("In %s ---------------------------------------LINE: %d\n",__FUNCTION__, __LINE__);
	DEBUG("readb bar2 register offset 0x280 = 0x%02x\n",readb(priv->data));

	//priv->data=bar2+0x280, SWRST offset=bar2+0x238.
	DEBUG("writeb SWRST reset...\n");
	writeb(1,(priv->data-0x80+0x38));


	// set DMA
	ret = dma_coerce_mask_and_coherent(priv->dev, DMA_BIT_MASK(24));
	if (ret) {
		dev_err(priv->dev, "Unable to set coherent dma mask: disabling DMA\n");
		dma = PARPORT_DMA_NONE;
	}
	
	
	// set some more private data in the process of "__parport_pc_probe_port"
	priv->ctr = 0xc;
	priv->ctr_writable = ~0x10;
	priv->ecr = 0;
	priv->ecr_writable = ecr_writable;
	priv->fifo_depth = 0;
	priv->dma_buf = NULL;
	priv->dma_handle = 0;
	INIT_LIST_HEAD(&priv->list);
	priv->pport = p;
	p->modes = PARPORT_MODE_PCSPP | PARPORT_MODE_SAFEININT;
	/* check process starts here
		1. parport_ECR_present
		2. parport_EPP_supported
		3. parport_SPP_supported
		4. parport_PS2_supported
	*/
	parport_ECR_present(p);
	if (!parport_EPP_supported(p))
		parport_ECPEPP_supported(p);

	if (!parport_SPP_supported(p))
		/* No port. */
		return -1;
	if (priv->ecr)
		parport_ECPPS2_supported(p);
	else
		parport_PS2_supported(p);

	p->size = (p->modes & PARPORT_MODE_EPP) ? 8 : 3;
	pr_info("%s: ARM-style", p->name);


	if (p->irq == PARPORT_IRQ_AUTO) {
		p->irq = PARPORT_IRQ_NONE;
		parport_irq_probe(p);
	} else if (p->irq == PARPORT_IRQ_PROBEONLY) {
		p->irq = PARPORT_IRQ_NONE;
		parport_irq_probe(p);
		probedirq = p->irq;
		p->irq = PARPORT_IRQ_NONE;
	}
	if (p->irq != PARPORT_IRQ_NONE) {
		pr_cont(", irq %d", p->irq);
		priv->ctr_writable |= 0x10;

		if (p->dma == PARPORT_DMA_AUTO) {
			p->dma = PARPORT_DMA_NONE;
			parport_dma_probe(p);
		}
	}
	if (p->dma == PARPORT_DMA_AUTO) /* To use DMA, giving the irq
					   is mandatory (see above) */
		p->dma = PARPORT_DMA_NONE;

	if (parport_ECP_supported(p) &&
	    p->dma != PARPORT_DMA_NOFIFO &&
	    priv->fifo_depth > 0 && p->irq != PARPORT_IRQ_NONE) {
		p->modes |= PARPORT_MODE_ECP | PARPORT_MODE_COMPAT;
		if (p->dma != PARPORT_DMA_NONE)
			p->modes |= PARPORT_MODE_DMA;
	} else
		/* We can't use the DMA channel after all. */
		p->dma = PARPORT_DMA_NONE;

	if ((p->modes & (PARPORT_MODE_ECP | PARPORT_MODE_COMPAT)) != 0) {
		if ((p->modes & PARPORT_MODE_DMA) != 0)
			pr_cont(", dma %d", p->dma);
		else
			pr_cont(", using FIFO");
	}

	/*****print HW supported Parport modes START****/
	pr_cont(" [");
#define printmode(x)							\
do {									\
	if (p->modes & PARPORT_MODE_##x)				\
		pr_cont("%s%s", f++ ? "," : "", #x);			\
} while (0)

	{
		int f = 0;
		printmode(PCSPP);
		printmode(TRISTATE);
		printmode(COMPAT);
		printmode(EPP);
		printmode(ECP);
		printmode(DMA);
	}
#undef printmode
	pr_cont("]\n");

	/*****print HW supported Parport modes END****/
	if (probedirq != PARPORT_IRQ_NONE)
		pr_info("%s: irq %d detected\n", p->name, probedirq);


	if (p->dma != PARPORT_DMA_NONE) {
		if (request_dma(p->dma, p->name)) {
			pr_warn("%s: dma %d in use, resorting to PIO operation\n",
				p->name, p->dma);
			p->dma = PARPORT_DMA_NONE;
		} else {
			priv->dma_buf =
			  dma_alloc_coherent(priv->dev,
					       PAGE_SIZE,
					       &priv->dma_handle,
					       GFP_KERNEL);
			if (!priv->dma_buf) {
				pr_warn("%s: cannot get buffer for DMA, resorting to PIO operation\n",
					p->name);
				free_dma(p->dma);
				p->dma = PARPORT_DMA_NONE;
			}
		}
	}

	/* Done probing.  Now put the port into a sensible start-up state. */
	if (priv->ecr)
		/*
		 * Put the ECP detected port in PS2 mode.
		 * Do this also for ports that have ECR but don't do ECP.
		 */
		ECR_WRITE(p, 0x34);

	parport_ax_write_data(p, 0);
	parport_ax_data_forward(p);

	/* Now that we've told the sharing engine about the port, and
	   found out its characteristics, let the high-level drivers
	   know about it. */
	spin_lock(&ports_lock);
	list_add(&priv->list, &ports_list);
	spin_unlock(&ports_lock);

	return 0;
}


#else
static LIST_HEAD(ports_list);
#endif

#if !PARPORT_PCI_BASE  //below are for iobase

#if LINUX_VERSION_CODE < KERNEL_VERSION(6,3,0)	
/* frob_control, but for ECR */
static void frob_econtrol(struct parport *pb, unsigned char m,
			   unsigned char v)
{
	unsigned char ectr = 0;

	if (m != 0xff)
		ectr = inb(ECONTROL(pb));

	pr_debug("frob_econtrol(%02x,%02x): %02x -> %02x\n",
		 m, v, ectr, (ectr & ~m) ^ v);

	outb((ectr & ~m) ^ v, ECONTROL(pb));
}

#else
/* frob_control, but for ECR */
static void frob_econtrol(struct parport *pb, unsigned char m,
			   unsigned char v)
{
	const struct parport_pc_private *priv = pb->physport->private_data;
	unsigned char ecr_writable = priv->ecr_writable;
	unsigned char ectr = 0;
	unsigned char new;

	if (m != 0xff)
		ectr = inb(ECONTROL(pb));

	new = (ectr & ~m) ^ v;
	if (ecr_writable)
		/* All known users of the ECR mask require bit 0 to be set. */
		new = (new & ecr_writable) | 1;

	pr_debug("frob_econtrol(%02x,%02x): %02x -> %02x\n", m, v, ectr, new);

	outb(new, ECONTROL(pb));
}

#endif

static inline void frob_set_mode(struct parport *p, int mode)
{
	frob_econtrol(p, ECR_MODE_MASK, mode << 5);
}

/*
 * Clear TIMEOUT BIT in EPP MODE
 *
 * This is also used in SPP detection.
 */
static int clear_epp_timeout(struct parport *pb)
{
	unsigned char r;

	if (!(parport_pc_read_status(pb) & 0x01))
		return 1;

	/* To clear timeout some chips require double read */
	parport_pc_read_status(pb);
	r = parport_pc_read_status(pb);
	outb(r | 0x01, STATUS(pb)); /* Some reset by writing 1 */
	outb(r & 0xfe, STATUS(pb)); /* Others by writing 0 */
	r = parport_pc_read_status(pb);

	return !(r & 0x01);
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(6,3,0)	
static size_t parport_pc_epp_read_data(struct parport *port, void *buf,
				       size_t length, int flags)
{
	size_t got = 0;

	if (flags & PARPORT_W91284PIC) {
		unsigned char status;
		size_t left = length;

		/* use knowledge about data lines..:
		 *  nFault is 0 if there is at least 1 byte in the Warp's FIFO
		 *  pError is 1 if there are 16 bytes in the Warp's FIFO
		 */
		status = inb(STATUS(port));

		while (!(status & 0x08) && got < length) {
			if (left >= 16 && (status & 0x20) && !(status & 0x08)) {
				/* can grab 16 bytes from warp fifo */
				if (!((long)buf & 0x03))
					insl(EPPDATA(port), buf, 4);
				else
					insb(EPPDATA(port), buf, 16);
				buf += 16;
				got += 16;
				left -= 16;
			} else {
				/* grab single byte from the warp fifo */
				*((char *)buf) = inb(EPPDATA(port));
				buf++;
				got++;
				left--;
			}
			status = inb(STATUS(port));
			if (status & 0x01) {
				/* EPP timeout should never occur... */
				printk(KERN_DEBUG "%s: EPP timeout occurred while talking to w91284pic (should not have done)\n",
				       port->name);
				clear_epp_timeout(port);
			}
		}
		return got;
	}
	if ((flags & PARPORT_EPP_FAST) && (length > 1)) {
		if (!(((long)buf | length) & 0x03))
			insl(EPPDATA(port), buf, (length >> 2));
		else
			insb(EPPDATA(port), buf, length);
		if (inb(STATUS(port)) & 0x01) {
			clear_epp_timeout(port);
			return -EIO;
		}
		return length;
	}
	for (; got < length; got++) {
		*((char *)buf) = inb(EPPDATA(port));
		buf++;
		if (inb(STATUS(port)) & 0x01) {
			/* EPP timeout */
			clear_epp_timeout(port);
			break;
		}
	}

	return got;
}

static size_t parport_pc_epp_write_data(struct parport *port, const void *buf,
					size_t length, int flags)
{
	size_t written = 0;

	if ((flags & PARPORT_EPP_FAST) && (length > 1)) {
		if (!(((long)buf | length) & 0x03))
			outsl(EPPDATA(port), buf, (length >> 2));
		else
			outsb(EPPDATA(port), buf, length);
		if (inb(STATUS(port)) & 0x01) {
			clear_epp_timeout(port);
			return -EIO;
		}
		return length;
	}
	for (; written < length; written++) {
		outb(*((char *)buf), EPPDATA(port));
		buf++;
		if (inb(STATUS(port)) & 0x01) {
			clear_epp_timeout(port);
			break;
		}
	}

	return written;
}

static size_t parport_pc_epp_read_addr(struct parport *port, void *buf,
					size_t length, int flags)
{
	size_t got = 0;

	if ((flags & PARPORT_EPP_FAST) && (length > 1)) {
		insb(EPPADDR(port), buf, length);
		if (inb(STATUS(port)) & 0x01) {
			clear_epp_timeout(port);
			return -EIO;
		}
		return length;
	}
	for (; got < length; got++) {
		*((char *)buf) = inb(EPPADDR(port));
		buf++;
		if (inb(STATUS(port)) & 0x01) {
			clear_epp_timeout(port);
			break;
		}
	}

	return got;
}

static size_t parport_pc_epp_write_addr(struct parport *port,
					 const void *buf, size_t length,
					 int flags)
{
	size_t written = 0;

	if ((flags & PARPORT_EPP_FAST) && (length > 1)) {
		outsb(EPPADDR(port), buf, length);
		if (inb(STATUS(port)) & 0x01) {
			clear_epp_timeout(port);
			return -EIO;
		}
		return length;
	}
	for (; written < length; written++) {
		outb(*((char *)buf), EPPADDR(port));
		buf++;
		if (inb(STATUS(port)) & 0x01) {
			clear_epp_timeout(port);
			break;
		}
	}

	return written;
}

static size_t parport_pc_ecpepp_read_data(struct parport *port, void *buf,
					  size_t length, int flags)
{
	size_t got;

	frob_set_mode(port, ECR_EPP);
	parport_pc_data_reverse(port);
	parport_pc_write_control(port, 0x4);
	got = parport_pc_epp_read_data(port, buf, length, flags);
	frob_set_mode(port, ECR_PS2);

	return got;
}

static size_t parport_pc_ecpepp_write_data(struct parport *port,
					   const void *buf, size_t length,
					   int flags)
{
	size_t written;

	frob_set_mode(port, ECR_EPP);
	parport_pc_write_control(port, 0x4);
	parport_pc_data_forward(port);
	written = parport_pc_epp_write_data(port, buf, length, flags);
	frob_set_mode(port, ECR_PS2);

	return written;
}

static size_t parport_pc_ecpepp_read_addr(struct parport *port, void *buf,
					  size_t length, int flags)
{
	size_t got;

	frob_set_mode(port, ECR_EPP);
	parport_pc_data_reverse(port);
	parport_pc_write_control(port, 0x4);
	got = parport_pc_epp_read_addr(port, buf, length, flags);
	frob_set_mode(port, ECR_PS2);

	return got;
}

static size_t parport_pc_ecpepp_write_addr(struct parport *port,
					    const void *buf, size_t length,
					    int flags)
{
	size_t written;

	frob_set_mode(port, ECR_EPP);
	parport_pc_write_control(port, 0x4);
	parport_pc_data_forward(port);
	written = parport_pc_epp_write_addr(port, buf, length, flags);
	frob_set_mode(port, ECR_PS2);

	return written;
}
#else
static size_t parport_pc_epp_read_data(struct parport *port, void *buf,
				       size_t length, int flags)
{
	size_t got = 0;

	if (flags & PARPORT_W91284PIC) {
		unsigned char status;
		size_t left = length;

		/* use knowledge about data lines..:
		 *  nFault is 0 if there is at least 1 byte in the Warp's FIFO
		 *  pError is 1 if there are 16 bytes in the Warp's FIFO
		 */
		status = inb(STATUS(port));

		while (!(status & 0x08) && got < length) {
			if (left >= 16 && (status & 0x20) && !(status & 0x08)) {
				/* can grab 16 bytes from warp fifo */
				if (!((long)buf & 0x03))
					insl(EPPDATA(port), buf, 4);
				else
					insb(EPPDATA(port), buf, 16);
				buf += 16;
				got += 16;
				left -= 16;
			} else {
				/* grab single byte from the warp fifo */
				*((char *)buf) = inb(EPPDATA(port));
				buf++;
				got++;
				left--;
			}
			status = inb(STATUS(port));
			if (status & 0x01) {
				/* EPP timeout should never occur... */
				printk(KERN_DEBUG "%s: EPP timeout occurred while talking to w91284pic (should not have done)\n",
				       port->name);
				clear_epp_timeout(port);
			}
		}
		return got;
	}
	if ((flags & PARPORT_EPP_FAST) && (length > 1)) {
		if (!(((long)buf | length) & 0x03))
			insl(EPPDATA(port), buf, (length >> 2));
		else
			insb(EPPDATA(port), buf, length);
		if (inb(STATUS(port)) & 0x01) {
			clear_epp_timeout(port);
			return -EIO;
		}
		return length;
	}
	for (; got < length; got++) {
		*((char *)buf) = inb(EPPDATA(port));
		buf++;
		if (inb(STATUS(port)) & 0x01) {
			/* EPP timeout */
			clear_epp_timeout(port);
			break;
		}
	}

	return got;
}

static size_t parport_pc_epp_write_data(struct parport *port, const void *buf,
					size_t length, int flags)
{
	size_t written = 0;

	if ((flags & PARPORT_EPP_FAST) && (length > 1)) {
		if (!(((long)buf | length) & 0x03))
			outsl(EPPDATA(port), buf, (length >> 2));
		else
			outsb(EPPDATA(port), buf, length);
		if (inb(STATUS(port)) & 0x01) {
			clear_epp_timeout(port);
			return -EIO;
		}
		return length;
	}
	for (; written < length; written++) {
		outb(*((char *)buf), EPPDATA(port));
		buf++;
		if (inb(STATUS(port)) & 0x01) {
			clear_epp_timeout(port);
			break;
		}
	}

	return written;
}

static size_t parport_pc_epp_read_addr(struct parport *port, void *buf,
					size_t length, int flags)
{
	size_t got = 0;

	if ((flags & PARPORT_EPP_FAST) && (length > 1)) {
		insb(EPPADDR(port), buf, length);
		if (inb(STATUS(port)) & 0x01) {
			clear_epp_timeout(port);
			return -EIO;
		}
		return length;
	}
	for (; got < length; got++) {
		*((char *)buf) = inb(EPPADDR(port));
		buf++;
		if (inb(STATUS(port)) & 0x01) {
			clear_epp_timeout(port);
			break;
		}
	}

	return got;
}

static size_t parport_pc_epp_write_addr(struct parport *port,
					 const void *buf, size_t length,
					 int flags)
{
	size_t written = 0;

	if ((flags & PARPORT_EPP_FAST) && (length > 1)) {
		outsb(EPPADDR(port), buf, length);
		if (inb(STATUS(port)) & 0x01) {
			clear_epp_timeout(port);
			return -EIO;
		}
		return length;
	}
	for (; written < length; written++) {
		outb(*((char *)buf), EPPADDR(port));
		buf++;
		if (inb(STATUS(port)) & 0x01) {
			clear_epp_timeout(port);
			break;
		}
	}

	return written;
}

static size_t parport_pc_ecpepp_read_data(struct parport *port, void *buf,
					  size_t length, int flags)
{
	size_t got;

	frob_set_mode(port, ECR_EPP);
	parport_pc_data_reverse(port);
	parport_pc_write_control(port, 0x4);
	got = parport_pc_epp_read_data(port, buf, length, flags);
	frob_set_mode(port, ECR_PS2);

	return got;
}

static size_t parport_pc_ecpepp_write_data(struct parport *port,
					   const void *buf, size_t length,
					   int flags)
{
	size_t written;

	frob_set_mode(port, ECR_EPP);
	parport_pc_write_control(port, 0x4);
	parport_pc_data_forward(port);
	written = parport_pc_epp_write_data(port, buf, length, flags);
	frob_set_mode(port, ECR_PS2);

	return written;
}

static size_t parport_pc_ecpepp_read_addr(struct parport *port, void *buf,
					  size_t length, int flags)
{
	size_t got;

	frob_set_mode(port, ECR_EPP);
	parport_pc_data_reverse(port);
	parport_pc_write_control(port, 0x4);
	got = parport_pc_epp_read_addr(port, buf, length, flags);
	frob_set_mode(port, ECR_PS2);

	return got;
}

static size_t parport_pc_ecpepp_write_addr(struct parport *port,
					    const void *buf, size_t length,
					    int flags)
{
	size_t written;

	frob_set_mode(port, ECR_EPP);
	parport_pc_write_control(port, 0x4);
	parport_pc_data_forward(port);
	written = parport_pc_epp_write_addr(port, buf, length, flags);
	frob_set_mode(port, ECR_PS2);

	return written;
}
#endif
#endif


static int parport99100_probe(struct pci_dev *dev, const struct pci_device_id *id){
	int err, irq;
#if PARPORT_PCI_BASE
	// unsigned long io_lo, io_hi;
	struct pci_parport_data *data;
	struct parport *pp;
	unsigned long	base, len;

	/* This is a PCI card */
	data = kmalloc(sizeof(struct pci_parport_data), GFP_KERNEL);
	if (!data){
		DEBUG("kmalloc FAILED\n");
		return -ENOMEM;
	}
		

	// spin_lock_init(&data->lock);
	memset(data,0,sizeof(struct pci_parport_data));
	err = pci_enable_device(dev);	
	if (err) {
		DEBUG("Device enable FAILED\n");
        return err;
	}

	
	// struct parport *pp;
	// #if defined(__x86_64__) || defined(__amd64__)
	/* bar2 */
	len =  pci_resource_len(dev, FL_BASE2);
	base = pci_resource_start(dev, FL_BASE2);
	data->mapbase = base;
	data->membase = ioremap(base,len);
	// io_lo = data->membase+0x280;
	// io_hi = data->membase+0x2a0;

	// save bar2 register offsets for later use...
	data->data		= 	data->membase + 0x280;
	data->status 	= 	data->membase + 0x284;
	data->ctrl 		= 	data->membase + 0x288;
	data->eppaddr 	= 	data->membase + 0x28c;
	data->eppdata 	= 	data->membase + 0x290;

	data->fifo		=	data->membase + 0x2a0;
	data->configa	=	data->membase + 0x2a0;
	data->configb	=	data->membase + 0x2a4;
	data->ectrl		=	data->membase + 0x2a8;

	// printk("readb bar2 register offset 0x280 = 0x%02x\n",readb(data->data));
	#if PARPORT_IRQ_ENABLE
    irq = dev->irq;
	#else
	irq = 0;
	#endif

    if (irq == IRQ_NONE) {
			printk(KERN_DEBUG
	// "PCI parallel port detected: %04x:%04x, I/O at %#lx(%#lx)\n",
	// 			id->vendor, id->device, io_lo, io_hi);
	"PCI parallel port detected: %04x:%04x, memI/O\n",
				id->vendor, id->device);
			irq = PARPORT_IRQ_NONE;
	} else {
			printk(KERN_DEBUG
	// "PCI parallel port detected: %04x:%04x, I/O at %#lx(%#lx), IRQ %d\n",
	// 			id->vendor, id->device, io_lo, io_hi, irq);
	"PCI parallel port detected: %04x:%04x, memI/O, IRQ %d\n",
				id->vendor, id->device, irq);
	}



	//***********************for replacing "parport_pc_probe_port" **********************
	pp = parport_register_port((unsigned long)data->membase+0x280, irq,
				   PARPORT_DMA_NONE,
				   &parport_ax99100pp_ops);

	// DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);			   
	if (pp == NULL) {
		dev_err(&dev->dev, "failed to register parallel port\n");
		err = -ENOMEM;
		goto err_disable;
	}

	pp->private_data = data;
	data->pport = pp;
	data->dev = &dev->dev;

	pci_set_drvdata(dev, pp);

	/* initialise the port controls */

	err=parport_ax_init_check(pp);


	if (irq >= 0) {
		/* request irq */
		err = request_irq(irq, parport_irq_handler,
				  IRQF_SHARED, "ax99100x_pp", pp);
		if (err < 0){
			dev_err(&dev->dev, "failed to request_irq\n");
			goto exit_port;
		}
	}

	parport_announce_port(pp);
#else
	unsigned long io_lo, io_hi;
	struct pci_parport_data *data;
	DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);
	/* This is a PCI card */
	data = devm_kzalloc(&dev->dev, sizeof(struct pci_parport_data), GFP_KERNEL);
	if (!data){
		dev_err(&dev->dev, "failed to devm_kzalloc\n");
		return -ENOMEM;
	}
		

	spin_lock_init(&data->lock);
	err = pci_enable_device(dev);
	if (err){
		dev_err(&dev->dev, "failed to pci_enable_device\n");
		return err;
	}
    io_lo = pci_resource_start(dev, FL_BASE0);
    io_hi = pci_resource_start(dev, FL_BASE1);

	#if PARPORT_IRQ_ENABLE
    irq = dev->irq;
	#else
	irq = 0;
	#endif
    
	if (irq == IRQ_NONE) {
			printk(KERN_DEBUG
	"PCI parallel port detected: %04x:%04x, I/O at %#lx(%#lx)\n",
				id->vendor, id->device, io_lo, io_hi);
			irq = PARPORT_IRQ_NONE;
	} else {
			printk(KERN_DEBUG
	"PCI parallel port detected: %04x:%04x, I/O at %#lx(%#lx), IRQ %d\n",
				id->vendor, id->device, io_lo, io_hi, irq);
	}
	data->pport =
		parport_pc_probe_port(io_lo, io_hi, irq,
				       PARPORT_DMA_NONE, &dev->dev,
				       IRQF_SHARED);
	if(data->pport==NULL){
		dev_err(&dev->dev, "failed to register parallel port\n");
		err=-1;
		kfree(data);
		goto err_disable;
	}
	
	/* Set up access functions to use ECP+EPP hardware. */
	data->pport->ops->epp_read_data = parport_pc_ecpepp_read_data;
	data->pport->ops->epp_write_data = parport_pc_ecpepp_write_data;
	data->pport->ops->epp_read_addr = parport_pc_ecpepp_read_addr;
	data->pport->ops->epp_write_addr = parport_pc_ecpepp_write_addr;

	pci_set_drvdata(dev, data);
	// dev_info(&dev->dev, "attached parallel port driver\n");
#endif

    return 0;	


err_disable:
	pci_disable_device(dev);
	return err;



#if PARPORT_PCI_BASE
 exit_port:
	parport_remove_port(pp);
//  exit_mem:
// 	kfree(data);
	return err;
#endif	

}



static void parport99100_remove(struct pci_dev *dev){
	// struct pci_parport_data *data = pci_get_drvdata(pdev);
	// pci_disable_device(pdev);
	struct pci_parport_data *data = pci_get_drvdata(dev);

	#if PARPORT_PCI_BASE
	parport_ax_unregister_port(data->pport);
	#else
	parport_pc_unregister_port(data->pport);
	#endif
	kfree(data);	
}





#define PCI_SUBVEN_ID_AX99100_PP	0x2000
#define PCI_SUBDEV_ID_AX99100	  	0xa000	

static struct pci_device_id parport99100_pci_tbl[] = {
	//{PCI_VENDOR_ID_NETMOS, PCI_ANY_ID, PCI_SUBDEV_ID_AX99100, PCI_SUBVEN_ID_AX99100, 0, 0, 0},	
	{0x125B, 0x9100, PCI_SUBDEV_ID_AX99100, PCI_SUBVEN_ID_AX99100_PP, 0, 0, 0},
	{0, },
};
MODULE_DEVICE_TABLE(pci, parport99100_pci_tbl);


/* PCI driver struct */
static struct pci_driver ax99100pp_pci_driver = {
	.name = "ax99100x_pp",
	.id_table = parport99100_pci_tbl,
	.probe = parport99100_probe,
	.remove = parport99100_remove,
};

/**
 * @brief This function is called, when the module is loaded into the kernel
 */
static int __init ax99100pp_init(void) {
	// printk("ax99100pp_pci_driver - Registering the PCIe device\n");
	return pci_register_driver(&ax99100pp_pci_driver);
}

/**
 * @brief This function is called, when the module is removed from the kernel
 */
static void __exit ax99100pp_exit(void) {
	// printk("ax99100pp_pci_driver - Unregistering the PCIe device\n");
	pci_unregister_driver(&ax99100pp_pci_driver);
}

module_init(ax99100pp_init);
module_exit(ax99100pp_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tony Chung <tonychung@asix.com.tw>");
MODULE_DESCRIPTION("AX99100x PCIE to Parallel port driver");

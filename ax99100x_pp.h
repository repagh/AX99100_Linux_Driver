#include <asm/io.h>

#define DEBUG_PARPORT			0 
#define DEBUG_MEM_MAP_PARPORT 	0


// ax99100x_pp.c DEBUG_MEM_MAP_PARPORT use.
#if DEBUG_MEM_MAP_PARPORT
#define DEBUG(fmt...)	printk(KERN_ERR fmt)
#else
#define DEBUG(fmt...)
#endif


struct pci_parport_data {
	unsigned char __iomem	*membase;	// bar2 
	resource_size_t		mapbase;	

	spinlock_t lock;

	struct parport *pport;
	struct pci_dev *pdev;
	struct resource		*io;
	struct device		*dev;

	/*********************for priv data************************/
	/* Contents of CTR. */
	unsigned char ctr;

	/* Bitmask of writable CTR bits. */
	unsigned char ctr_writable;

	/* Whether or not there's an ECR. */
	int ecr;

	/* Bitmask of writable ECR bits. */
	unsigned char ecr_writable;

	/* Number of PWords that FIFO will hold. */
	int fifo_depth;

	/* Number of bytes per portword. */
	int pword;

	/* Not used yet. */
	int readIntrThreshold;
	int writeIntrThreshold;

	/* buffer suitable for DMA, if DMA enabled */
	char *dma_buf;
	dma_addr_t dma_handle;
	struct list_head list;
	/*********************************************/

	// AX99100 Parport registers
	void __iomem		*data;    
	void __iomem		*status;  
	void __iomem		*ctrl;    
	void __iomem		*eppaddr; 
	void __iomem		*eppdata; 

	void __iomem		*fifo;    
	void __iomem		*configa; 
	void __iomem		*configb;
	void __iomem		*ectrl;   
	
};

/******************************************
********* Some Help Functions *************
******************************************/
static inline struct pci_parport_data *pp_to_drv(struct parport *p)
{
	
	return p->private_data;
}

/******************************************
*********Modified from parport_pc.h********
******************************************/
/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_PARPORT_PC_H
#define __LINUX_PARPORT_PC_H


static __inline__ void parport_ax_write_data(struct parport *p, unsigned char d)
{
    struct pci_parport_data *dd = p->private_data;
    
    writeb(d, dd->data);
	DEBUG("WrDat:%02x\n", d);
}

static __inline__ unsigned char parport_ax_read_data(struct parport *p)
{
	struct pci_parport_data *dd = pp_to_drv(p);
	unsigned char val = readb(dd->data);
	
	DEBUG("RdDat%02x\n", val);
	return val;
}

#if DEBUG_PARPORT
static inline void dump_parport_state (char *str, struct parport *p)
{
	const struct pci_parport_data *priv = pp_to_drv(p);
	DEBUG("In %s ---------------------------------------LINE: %d\n",__FUNCTION__, __LINE__);
    /* here's hoping that reading these ports won't side-effect anything underneath */
	unsigned char ecr = readb(priv->ectrl);//inb (ECONTROL (p));
	unsigned char dcr = readb(priv->ctrl);//inb (CONTROL (p));
	unsigned char dsr = readb(priv->status);//inb (STATUS (p));
	static const char *const ecr_modes[] = {"SPP", "PS2", "PPFIFO", "ECP", "xXx", "yYy", "TST", "CFG"};
	int i;

	printk (KERN_DEBUG "*** parport state (%s): ecr=[%s", str, ecr_modes[(ecr & 0xe0) >> 5]);
	if (ecr & 0x10) printk (",nErrIntrEn");
	if (ecr & 0x08) printk (",dmaEn");
	if (ecr & 0x04) printk (",serviceIntr");
	if (ecr & 0x02) printk (",f_full");
	if (ecr & 0x01) printk (",f_empty");
	for (i=0; i<2; i++) {
		printk ("]  dcr(%s)=[", i ? "soft" : "hard");
		dcr = i ? priv->ctr : readb(priv->ctrl);//inb (CONTROL (p));
	
		if (dcr & 0x20) {
			printk ("rev");
		} else {
			printk ("fwd");
		}
		if (dcr & 0x10) printk (",ackIntEn");
		if (!(dcr & 0x08)) printk (",N-SELECT-IN");
		if (dcr & 0x04) printk (",N-INIT");
		if (!(dcr & 0x02)) printk (",N-AUTOFD");
		if (!(dcr & 0x01)) printk (",N-STROBE");
	}
	printk ("]  dsr=[");
	if (!(dsr & 0x80)) printk ("BUSY");
	if (dsr & 0x40) printk (",N-ACK");
	if (dsr & 0x20) printk (",PERROR");
	if (dsr & 0x10) printk (",SELECT");
	if (dsr & 0x08) printk (",N-FAULT");
	printk ("]\n");
	DEBUG("In %s ---------------------------------------LINE: %d\n",__FUNCTION__, __LINE__);
	return;
}
#endif	/* !DEBUG_PARPORT */

/* __parport_pc_frob_control differs from parport_pc_frob_control in that
 * it doesn't do any extra masking. */

static __inline__ unsigned char __parport_ax_frob_control (struct parport *p,
							   unsigned char mask,
							   unsigned char val)
{
	struct pci_parport_data *priv = pp_to_drv(p);
	unsigned char ctr = priv->ctr;

	ctr = readb(priv->ctrl);
	ctr = (ctr & ~mask) | (val & mask);
	ctr &= priv->ctr_writable; /* only write writable bits. */
	writeb(ctr, priv->ctrl);//outb (ctr, CONTROL (p));
	priv->ctr = ctr;	/* Update soft copy */
	return ctr;
}

static __inline__ void parport_ax_data_reverse (struct parport *p)
{

	__parport_ax_frob_control (p, 0x20, 0x20);
	DEBUG("DIR=1\n");
}

static __inline__ void parport_ax_data_forward (struct parport *p)
{
	__parport_ax_frob_control (p, 0x20, 0x00);
	DEBUG("DIR=0\n");
}

static __inline__ void parport_ax_write_control (struct parport *p,
						 unsigned char d)
{
	const unsigned char wm = (PARPORT_CONTROL_STROBE |
				  PARPORT_CONTROL_AUTOFD |
				  PARPORT_CONTROL_INIT |
				  PARPORT_CONTROL_SELECT);
	/* Take this out when drivers have adapted to newer interface. */
	if (d & 0x20) {
		printk (KERN_DEBUG "%s (%s): use data_reverse for this!\n",
			p->name, p->cad->name);
		parport_ax_data_reverse (p);
	}

	__parport_ax_frob_control (p, wm, d);
	DEBUG("WrDCR,m=%02x,v=%02x\n", wm, d);
}

static __inline__ unsigned char parport_ax_read_control(struct parport *p)
{
	struct pci_parport_data *priv = pp_to_drv(p);
	const unsigned char rm = (PARPORT_CONTROL_STROBE |
				  PARPORT_CONTROL_AUTOFD |
				  PARPORT_CONTROL_INIT |
				  PARPORT_CONTROL_SELECT);

	priv->ctr = readb(priv->ctrl);
	DEBUG("RdDCR,v=%02x\n", (priv->ctr & rm));
	return (priv->ctr & rm); /* Use soft copy */
}

static __inline__ unsigned char parport_ax_frob_control (struct parport *p,
							 unsigned char mask,
							 unsigned char val)
{
	const unsigned char wm = (PARPORT_CONTROL_STROBE |
				  PARPORT_CONTROL_AUTOFD |
				  PARPORT_CONTROL_INIT |
				  PARPORT_CONTROL_SELECT);

	/* Take this out when drivers have adapted to newer interface. */
	if (mask & 0x20) {
		printk (KERN_DEBUG "%s (%s): use data_%s for this!\n",
			p->name, p->cad->name,
			(val & 0x20) ? "reverse" : "forward");
		if (val & 0x20) {
			parport_ax_data_reverse (p);
		} else {
			parport_ax_data_forward (p);
		}
	}

	/* Restrict mask and val to control lines. */
	mask &= wm;
	val = __parport_ax_frob_control (p, mask, val);
	DEBUG("MfDCR,m=%02x,v=%02x\n", mask, val);
	return val;
}

static __inline__ unsigned char parport_ax_read_status(struct parport *p)
{
	unsigned char stu;
#if DEBUG_MEM_MAP_PARPORT	
	static unsigned char status[2];
#endif	
	struct pci_parport_data *priv = pp_to_drv(p);	

	stu = readb(priv->status);
#if DEBUG_MEM_MAP_PARPORT
	status[0] = stu;
	if (status[0] != status [1])
	{
		DEBUG("RdDSR=%02x\n", stu);
		status[1] = status[0];
	}
#endif
	return stu;
}


static __inline__ void parport_ax_disable_irq(struct parport *p)
{
	__parport_ax_frob_control (p, 0x10, 0x00);
	DEBUG("INTA=0\n");
}

static __inline__ void parport_ax_enable_irq(struct parport *p)
{
	__parport_ax_frob_control (p, 0x10, 0x10);
	DEBUG("INTA=1\n");
}

/* Function prototypes */
void parport_ax_unregister_port(struct parport *p);

// extern void parport_pc_release_resources(struct parport *p);

// extern int parport_pc_claim_resources(struct parport *p);

// /* PCMCIA code will want to get us to look at a port.  Provide a mechanism. */
// extern struct parport *parport_pc_probe_port(unsigned long base,
// 					     unsigned long base_hi,
// 					     int irq, int dma,
// 					     struct device *dev,
// 					     int irqflags);
// extern void parport_pc_unregister_port(struct parport *p);

#endif
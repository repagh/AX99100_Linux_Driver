// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for ax99100 PCI-SPI multi-interface device
 *
 * Copyright (C) 2018-2019 T-platforms JSC (fancer.lancer@gmail.com)
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/spi/spi-mem.h>
#include <linux/spi/flash.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>
#include <linux/version.h>

#include "spi-ax99100.h"
/*
 * ax99100 PCI device
 *
 * Asix ax99100 chip is multi-IO PCIe device, which provides an access
 * to Local Bus, Parallel and Serial Ports, SPI, i2c and GPIO interfaces
 * over single PCIe endpoint. The availability of certain ports depends on
 * the chip mode (CHIP_MODE[2-0] pins state). In case if CHIP_MODE={110,100}
 * the chip exposes the SPI-interface at function 3, while functions 0 and 1
 * turned to be the Serial Ports (see 82500_ax99100 driver). i2c and GPIO
 * interfaces are accessible in any chip mode, but GPIO interrupts are
 * available for function 0 only. That's why GPIO interface support code is
 * implemented in 8250_ax99100 driver.
 */

/*
 * ax99100 PCI-SPI driver
 *
 * SPI-interface configuration registers are exposed by BAR0 of PCIe config
 * space, BAR1 is used for DMA setup and BAR5 provides an access to common
 * interfaces like i2c/GPIO (use 8250_ax99100 driver to access GPIOs). ax99100
 * IO-controller provides a features reach SPI interface like four SPI modes,
 * CS polarity inversion, LSB mode, clock source and divider settings, various
 * delays, fragmented transactions, embedded DMA engine. Of course we don't
 * use all of them for the driver implementation due to compatibility reason.
 * Particularly since the device FIFO is limited by 8 bytes only we use it
 * for small transfers of size less than FIFO size, the rest of transactions
 * are performed via DMA-engine.
 *
 * ax99100 PCI-SPI driver is created on top of the generic implementation of
 * transfer_one_message() provided by the SPI core. In particular,
 * the developed callback methods perform the following functionality:
 * - prepare_transfer_hardware   - enable SPI master controller
 * - unprepare_transfer_hardware - disable SPI master controller
 * - set_cs - select/deselect the corresponding CS pin in EDE-less config
 * - transfer_one - set transaction mode, LSB and speed setting, then submit
 *                  it for execution by device DMA engine.
 * - can_dma      - limit DMA mapping for transfers with size greater than FIFO
 *                  length.
 * - handle_err - reset SPI-controller in case of an error.
 *
 * Non of synchronization primitives are used since transfer operations are
 * performed by a single non-reentrant kernel thread.
 *
 * TODO
 * 1) Create an adaptive SPI baud rate algo by switching between available
 * clock sources: 125MHz internal PLL, 100MHz PCIe reference clock and custom
 * external clock - EXT_CLK. Internal PLL source is currently used by this
 * driver, which practically gives at most 62.5 MHz SPI clocks.
 * 2) Add an external chip-select decoder support. SPI-controller provides a way
 * to handle the external decoder connected to the SS pins (like sn74as138).
 * The feature is activated by setting SPI_SSOL_EDE bit allows to increase
 * a maximum number of SPI-slaves from 3 to 7.
 */
static inline u8 ax99100_spi_read(struct ax99100_data *ax, unsigned long addr)
{
	// return ioread8(ax->pci.sm + addr);
#if SPI_REG_BAR
	// ax->pci.dm = bars[1];
	return ioread8(ax->pci.dm + addr);
#else
	// ax->pci.sm = bars[0];
	return ioread8(ax->pci.sm + addr);
#endif
}

static inline void ax99100_spi_write(struct ax99100_data *ax, u8 val,
				     unsigned long addr)
{
	// iowrite8(val, ax->pci.sm + addr);
#if SPI_REG_BAR
	// ax->pci.dm = bars[1];
	iowrite8(val, ax->pci.dm + addr);
#else
	// ax->pci.sm = bars[0];
	iowrite8(val, ax->pci.sm + addr);
#endif
}

static inline u32 ax99100_dma_read(struct ax99100_data *ax, unsigned long addr)
{
	return ioread32(ax->pci.dm + addr);
}

static inline void ax99100_dma_write(struct ax99100_data *ax, u32 val,
				     unsigned long addr)
{
	iowrite32(val, ax->pci.dm + addr);
}

static void ax99100_spi_handle_err(struct spi_controller *ctrl,
				   struct spi_message *msg)
{
	struct ax99100_data *ax = spi_controller_get_devdata(ctrl);

	ax99100_dma_write(ax, 1, SPI_SWRST);
	ax->spi.sclk = 0;
}

static int ax99100_spi_prep_hw(struct spi_controller *ctlr)
{
	struct ax99100_data *ax = spi_controller_get_devdata(ctlr);

	/* Just enable SPI master controller and CS signals for now. */
	ax99100_spi_write(ax, SPI_CMR_MEN | SPI_CMR_SSOE, SPI_CMR);

	return 0;
}

static int ax99100_spi_unprep_hw(struct spi_controller *ctlr)
{
	struct ax99100_data *ax = spi_controller_get_devdata(ctlr);

	/* Disable SPI master controller and put CS signals to tri-state. */
	ax99100_spi_write(ax, 0, SPI_CMR);

	return 0;
}

static void ax99100_spi_set_cs(struct spi_device *spi, bool is_high)
{
	struct ax99100_data *ax = spi_controller_get_devdata(spi->controller);
	u8 ssol = 0x7;

	/* Since EDE isn't supported just set the shifted SS level. */
	if (!is_high){
		#if LINUX_VERSION_CODE < KERNEL_VERSION(6,8,0)
		ssol &= ~(1 << (spi->chip_select));
		#else
		ssol &= ~(1 << spi_get_chipselect(spi,0));
		#endif
	}

	ax99100_spi_write(ax, SPI_SSOL_SS(ssol), SPI_SSOL);
}

static void ax99100_spi_set_mode(struct ax99100_data *ax,
				 struct spi_device *spi)
{
	u8 cmr = SPI_CMR_MEN | SPI_CMR_SSOE;

	/* bits_per_word setting should have been verified by the SPI core. */

	cmr |= SPI_CMR_MODE(spi->mode);

	if (spi->mode & SPI_LSB_FIRST)
		cmr |= SPI_CMR_LSB;

	ax99100_spi_write(ax, cmr, SPI_CMR);
}

static void ax99100_spi_set_clk(struct ax99100_data *ax, struct spi_transfer *t)
{
	u8 divider;
	u32 sclk;

	/* Calculate the source clocks divider so the max frequency would
	 * be limited to 62.5MHz.
	 */
	divider = DIV_ROUND_UP(SPI_CKS125_MAX_FREQ, t->speed_hz);
	divider = clamp_t(u8, divider, 2, 255);

	sclk = SPI_CKS125_MAX_FREQ / divider;
	if (sclk == ax->spi.sclk)
		return;

	/* Disable, set then enable the clocks source selector. */
	ax->spi.sclk = sclk;
	ax99100_spi_write(ax, SPI_CSS_CKS_125MHZ, SPI_CSS);
	ax99100_spi_write(ax, divider, SPI_BRR);
	ax99100_spi_write(ax, SPI_CSS_CKS_125MHZ | SPI_CSS_DIVEN, SPI_CSS);
}

static bool ax99100_spi_can_dma(struct spi_controller *ctlr,
				struct spi_device *spi,
				struct spi_transfer *t)
{
	/* Perform DMA only for the transfers with length greater than FIFO.
	 * Note that the embedded DMA engine supports transfer up to a single
	 * byte, but we don't want to use it for small transactions since
	 * mapping might consume much more CPU resources than the transaction
	 * execution.
	 */
	if (t->len <= SPI_MAX_FIFO_LEN)
		return false;

	return true;
}

static int ax99100_spi_sync_wait(struct ax99100_data *ax, unsigned int len)
{
	unsigned long long ms = 1;

	ms = 8LL * 1000LL * len;
	do_div(ms, ax->spi.sclk);
	ms += ms + 200; /* some tolerance */
	if (ms > UINT_MAX)
		ms = UINT_MAX;

	ms = wait_for_completion_timeout(&ax->spi.xfer_completion,
					 msecs_to_jiffies(ms));
	if (ms == 0) {
		dev_err(&ax->spi.ctlr->cur_msg->spi->dev,
			"SPI transfer timed out\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int ax99100_spi_sync_fifo(struct ax99100_data *ax, const u8 *tx_buf,
				 u8 *rx_buf, unsigned int len)
{
	u8 sdcr = SPI_DCR_OPCFE | SPI_DCR_DMA_GO |
		  SPI_DCR_TCI_EN | SPI_DCR_TERRI_EN;
	unsigned int i;
	int ret;
	u8 ssol;

	/* Push data to FIFO if transmission buffer is specified. */
	if (tx_buf) {
		for (i = 0; i < len; ++i)
			ax99100_spi_write(ax, tx_buf[i], SPI_TOF0 + i);
	} else {
		for (i = 0; i < len; ++i)
			ax99100_spi_write(ax, 0, SPI_TOF0 + i);
	}

	/* Setup transaction length. We are sure it is within [1,8] range. */
	ssol = (ax99100_spi_read(ax, SPI_SSOL) & ~GENMASK(6, 4)) |
		SPI_SSOL_STOL(len - 1);
	ax99100_spi_write(ax, ssol, SPI_SSOL);

	/* Submit the transaction. */
	ax99100_spi_write(ax, sdcr, SPI_DCR);

	/* Wait for the transaction to be finished. */
	ret = ax99100_spi_sync_wait(ax, len);
	if (ret)
		return ret;

	/* Collect the retrieved data if rx_buf is specified. */
	if (rx_buf) {
		for (i = 0; i < len; ++i)
			rx_buf[i] = ax99100_spi_read(ax, SPI_TOF0 + i);
	}

	return 0;
}

static int ax99100_spi_send_fifo(struct ax99100_data *ax,
				 struct spi_transfer *t)
{
	unsigned int len, leftover, base;
	const void *tx_buf;
	void *rx_buf;
	int ret;

	for (base = 0; base < t->len; base += len) {
		leftover = t->len - base;
		len = min(leftover, SPI_MAX_FIFO_LEN);

		tx_buf = t->tx_buf ? &((u8 *)t->tx_buf)[base] : NULL;
		rx_buf = t->rx_buf ? &((u8 *)t->rx_buf)[base] : NULL;

		ret = ax99100_spi_sync_fifo(ax, tx_buf, rx_buf, len);
		if (ret)
			return ret;
	}

	return 0;
}

static int ax99100_spi_sync_dma(struct ax99100_data *ax, dma_addr_t tx_dma,
				dma_addr_t rx_dma, unsigned int len)
{
	u8 sdcr = SPI_DCR_DMA_GO | SPI_DCR_TCI_EN | SPI_DCR_TERRI_EN;

	/* Mapping has been done either by a protocol driver or by the SPI
	 * core. Here we just initialize the DMA-engine and execute the
	 * transaction.
	 */
	if (tx_dma) {
		sdcr |= SPI_DCR_ETDMA;

		ax99100_dma_write(ax, (u32)tx_dma, TDMA_SAR0);
		ax99100_dma_write(ax, (u32)(tx_dma >> 32), TDMA_SAR1);
		ax99100_dma_write(ax, TDMA_LR_LEN(len), TDMA_LR);
		ax99100_dma_write(ax, TDMA_STAR_START, TDMA_STAR);
	} else {
		ax99100_dma_write(ax, 0, TDMA_SAR0);
		ax99100_dma_write(ax, 0, TDMA_SAR1);
		ax99100_dma_write(ax, 0, TDMA_LR);
	}

	if (rx_dma) {
		sdcr |= SPI_DCR_ERDMA;

		ax99100_dma_write(ax, (u32)rx_dma, RDMA_SAR0);
		ax99100_dma_write(ax, (u32)(rx_dma >> 32), RDMA_SAR1);
		ax99100_dma_write(ax, RDMA_LR_LEN(len), RDMA_LR);
		ax99100_dma_write(ax, RDMA_STAR_START, RDMA_STAR);
	} else {
		ax99100_dma_write(ax, 0, RDMA_SAR0);
		ax99100_dma_write(ax, 0, RDMA_SAR1);
		ax99100_dma_write(ax, 0, RDMA_LR);
	}

	/* Submit the transaction. */
	ax99100_spi_write(ax, sdcr, SPI_DCR);

	/* Wait for the transaction to be finished. */
	return ax99100_spi_sync_wait(ax, len);
}

static int ax99100_spi_send_dma_tx(struct ax99100_data *ax,
				   struct spi_transfer *t)
{
	struct scatterlist *sg;
	unsigned int i;
	int ret;

	for_each_sg(t->tx_sg.sgl, sg, t->tx_sg.nents, i) {
		ret = ax99100_spi_sync_dma(ax, sg_dma_address(sg),
					   0, sg_dma_len(sg));
		if (ret)
			return ret;
	}

	return 0;
}

static int ax99100_spi_send_dma_rx(struct ax99100_data *ax,
				   struct spi_transfer *t)
{
	struct scatterlist *sg;
	unsigned int i;
	int ret;

	for_each_sg(t->rx_sg.sgl, sg, t->rx_sg.nents, i) {
		ret = ax99100_spi_sync_dma(ax, 0, sg_dma_address(sg),
					   sg_dma_len(sg));
		if (ret)
			return ret;
	}

	return 0;
}

static int ax99100_spi_send_dma_both(struct ax99100_data *ax,
				     struct spi_transfer *t)
{
	struct scatterlist *tx_sg, *rx_sg;
	dma_addr_t tx_dma = 0, rx_dma = 0;
	unsigned int tl = 0, rl = 0;
	unsigned int base, len;
	int ti = -1, ri = -1;
	int ret;

	/* ax99100 DMA seems to work as expected for SPI only if the RX and TX
	 * buffers lengths match. Documentation though vaguely but states this.
	 * Additionally as we noticed that the TCI interrupt will be raised
	 * only when both transactions are finished if both are enabled (when
	 * DMA_GO is switched to zero). Seeing the RX/TX SG-table entries in
	 * general may have different lengths, we need to work this DMA
	 * peculiarity around. Lets virtually split the SG-lists to the set
	 * of transfers, which length is minimum of the ordered SG-entries
	 * lengths. This shall work since the total length of the SPI transfer
	 * matches the total lengths of the TX and RX SG-lists transactions.
	 * An ASCII-sketch of the implemented algo is following:
	 *                   t->len
	 *                |___________|
	 * tx_sg list:    |___|____|__|
	 * rx_sg list:    |_|____|____|
	 * DMA transfers: |_|_|__|_|__|
	 *
	 */
	for (base = 0, len = 0; base < t->len; base += len) {
		if (tl) {
			tx_dma += len;
		} else {
			tx_sg = &t->tx_sg.sgl[++ti];
			tx_dma = sg_dma_address(tx_sg);
			tl = sg_dma_len(tx_sg);
		}
		if (rl) {
			rx_dma += len;
		} else {
			rx_sg = &t->rx_sg.sgl[++ri];
			rx_dma = sg_dma_address(rx_sg);
			rl = sg_dma_len(rx_sg);
		}

		len = min(tl, rl);

		ret = ax99100_spi_sync_dma(ax, tx_dma, rx_dma, len);
		if (ret)
			return ret;

		tl -= len;
		rl -= len;
	}

	return 0;
}

static int ax99100_spi_send_dma(struct ax99100_data *ax,
				struct spi_transfer *t)
{
	int ret;

	if (ax->spi.ctlr->cur_msg->is_dma_mapped)
		return ax99100_spi_sync_dma(ax, t->tx_dma, t->rx_dma, t->len);

	/* For the sake of the code simplification perform different methods
	 * for half and full duplex transfers.
	 */
	if (!t->rx_sg.nents)
		ret = ax99100_spi_send_dma_tx(ax, t);
	else if (!t->tx_sg.nents)
		ret = ax99100_spi_send_dma_rx(ax, t);
	else
		ret = ax99100_spi_send_dma_both(ax, t);

	return ret;
}

static void ax99100_spi_stop(struct ax99100_data *ax)
{
	/* Just stop both TX and RX DMA transfers. */
	ax99100_dma_write(ax, TDMA_STPR_STOP, TDMA_STPR);
	ax99100_dma_write(ax, RDMA_STPR_STOP, RDMA_STPR);

	udelay(10);
}

static int ax99100_spi_transfer_one(struct spi_controller *ctlr,
				    struct spi_device *spi,
				    struct spi_transfer *t)
{
	struct ax99100_data *ax = spi_controller_get_devdata(ctlr);

	ax99100_spi_set_mode(ax, spi);

	ax99100_spi_set_clk(ax, t);

	if ((ctlr->cur_msg_mapped || ctlr->cur_msg->is_dma_mapped) &&
	    ax99100_spi_can_dma(ctlr, spi, t))
		return ax99100_spi_send_dma(ax, t);

	return ax99100_spi_send_fifo(ax, t);
}

static irqreturn_t ax99100_spi_isr(int irq, void *devid)
{
	struct ax99100_data *ax = devid;
	irqreturn_t ret = IRQ_NONE;
	u32 tdma_sr, rdma_sr;
	u8 smisr;

	/* Find out the cause of the IRQ. */
	smisr = ax99100_spi_read(ax, SPI_MISR);
	if (smisr & SPI_MISR_TERR) {
		tdma_sr = ax99100_dma_read(ax, TDMA_SR);
		rdma_sr = ax99100_dma_read(ax, RDMA_SR);

		dev_err(&ax->spi.ctlr->dev, "Transfer failed: tx %x, rx %x\n",
			tdma_sr, rdma_sr);

		ret = IRQ_HANDLED;
	}
	if (smisr & SPI_MISR_TC) {
		complete(&ax->spi.xfer_completion);

		ret = IRQ_HANDLED;
	}
	if (ret == IRQ_HANDLED)
		ax99100_spi_write(ax, smisr, SPI_MISR);

	return ret;
}
#if SPI_FLASH_DEV
static void ax_spi_fill_partition(struct mtd_partition *part)
{
	memset(part, 0, sizeof(*part));
	/* Start from the mandatory descriptor region */
	part->size = 4000000;
	part->name = "mx25r3235f";

}
#endif

static int ax99100_spi_init(struct ax99100_data *ax)
{
	struct spi_controller *ctlr;
	struct pci_dev *pdev = ax->pci.pdev;
	#if SPI_FLASH_DEV
	struct flash_platform_data *pdata;
	struct spi_board_info chip;
	#endif
	int ret;

	/* Reset SPI interface before using it. */
	ax99100_dma_write(ax, 1, SPI_SWRST);

	ctlr = spi_alloc_master(&pdev->dev, 0);
	if (!ctlr) {
		dev_err(&pdev->dev, "Memory allocation failed for SPI\n");
		return -ENOMEM;
	}

	/* ax99100 DMA supports 64-bit addresses, so try 64-bits DMA mask first.
	 * If it fails, then fall back to the 32-bits DMA addresses
	 */
	ret = dma_coerce_mask_and_coherent(&ctlr->dev, DMA_BIT_MASK(64));
	if (ret) {
		ret = dma_set_mask_and_coherent(&ctlr->dev, DMA_BIT_MASK(32));
		if (ret) {
			dev_err(&pdev->dev, "Couldn't set DMA mask\n");
			goto err_spi_free_master;
		}
	}

	ax->spi.ctlr = ctlr;
	init_completion(&ax->spi.xfer_completion);
	ctlr->bus_num = -1;
	ctlr->num_chipselect = 3;
	ctlr->mode_bits = SPI_CPOL | SPI_CPHA | SPI_NO_CS | SPI_LSB_FIRST;
	ctlr->bits_per_word_mask = BIT(7);
	ctlr->max_dma_len = SPI_MAX_DMA_LEN;
	ctlr->min_speed_hz = SPI_CKS125_MAX_FREQ / 255;
	ctlr->max_speed_hz = SPI_CKS125_MAX_FREQ / 4;
	ctlr->flags = 0;
	ctlr->can_dma = ax99100_spi_can_dma;
	ctlr->handle_err = ax99100_spi_handle_err;
	ctlr->prepare_transfer_hardware = ax99100_spi_prep_hw;
	ctlr->unprepare_transfer_hardware = ax99100_spi_unprep_hw;
	ctlr->set_cs = ax99100_spi_set_cs;
	ctlr->transfer_one = ax99100_spi_transfer_one;

	spi_controller_set_devdata(ctlr, ax);

	ret = devm_spi_register_controller(&pdev->dev, ctlr);
	if (ret) {
		dev_err(&pdev->dev, "Couldn't register SPI controller\n");
		goto err_spi_free_master;
	}

	ret = devm_request_irq(&pdev->dev, ax->pci.irq, ax99100_spi_isr,
			       IRQF_ONESHOT | IRQF_SHARED, "ax99100_spi", ax);
	if (ret) {
		dev_err(&pdev->dev, "Failed to set IRQ#%d handler\n",
			ax->pci.irq);
		goto err_spi_free_master;
	}

	dev_info(&pdev->dev, "SPI-%d interface created\n", ctlr->bus_num);

	// SPI Dev starts here
	#if SPI_FLASH_DEV
	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->nr_parts = 1;
	pdata->parts = devm_kcalloc(&pdev->dev, pdata->nr_parts,
				    sizeof(*pdata->parts), GFP_KERNEL);
	if (!pdata->parts)
		return -ENOMEM;
	
	ax_spi_fill_partition(pdata->parts);
	
	memset(&chip, 0, sizeof(chip));
	snprintf(chip.modalias, sizeof(chip.modalias), "spi-nor");
	chip.platform_data = pdata;
	chip.chip_select = 0;
	chip.mode = SPI_MODE_3;
	if (!spi_new_device(ctlr, &chip))
		return -ENODEV;
	#endif

	return 0;



err_spi_free_master:
	spi_controller_put(ctlr);

	return ret;
}

static void ax99100_spi_clear(struct ax99100_data *ax)
{
	ax99100_spi_stop(ax);

	/* Manually free IRQ otherwise PCI free irq vectors will fail */
	devm_free_irq(&ax->pci.pdev->dev, ax->pci.irq, ax);
}

/*
 * ax99100 PCI-i2c interface driver
 *
 * ax99100 chips provides a functionally limited smbus-interface. It is made
 * mostly for EEPROM read/write operations and provides just SMBus
 * byte-read/byte-write/word-write data operations. Even though the operations
 * set is very limited it shall be enough to access some smbus-sensors,
 * i2c-muxes, GPIO-expanders.
 */
#if CONFIG_SPI_ax99100_I2C
static inline u32 ax99100_i2c_read(struct ax99100_data *ax, unsigned long addr)
{
	return ioread32(ax->pci.im + addr);
}

static inline void ax99100_i2c_write(struct ax99100_data *ax, u32 val,
				     unsigned long addr)
{
	iowrite32(val, ax->pci.im + addr);
}

static int ax99100_i2c_xfer(struct i2c_adapter *adapter,
		u16 addr, unsigned short flags, char read_write, u8 command,
		int size, union i2c_smbus_data *data)
{
	struct ax99100_data *ax = adapter->algo_data;
	u32 sdacr = 0, sclcr;
	int itr;

	sclcr = ax99100_i2c_read(ax, I2C_SCLCR) & ~I2C_SCLCR_ADR_LSB(1);

	/* Collect the transfer settings */
	switch (size) {
	case I2C_SMBUS_BYTE_DATA:
		sdacr |= I2C_CR_MA(command) | I2C_CR_DATA(data->byte);
		break;
	case I2C_SMBUS_WORD_DATA:
		if (read_write == I2C_SMBUS_READ) {
			dev_warn(&adapter->dev, "Unsupported read-word op\n");
			return -EOPNOTSUPP;
		}
		sdacr |= I2C_CR_MAF |
			 I2C_CR_MA(((u16)command << 8) | (data->word & 0xFF)) |
			 I2C_CR_DATA(data->word >> 8);
		break;
	default:
		dev_warn(&adapter->dev, "Unsupported transaction %d\n", size);
		return -EOPNOTSUPP;
	}

	if (read_write == I2C_SMBUS_WRITE)
		sdacr |= I2C_CR_RW_IND;

	sdacr |= I2C_CR_ADR(addr >> 1);
	sclcr |= I2C_SCLCR_ADR_LSB(addr & 0x1);

	/* Execute the SMBus transaction over the i2c interface. We check
	 * the transaction status at most I2C_NUM_ITER times until give up
	 * and return an error.
	 */
	ax99100_i2c_write(ax, sclcr, I2C_SCLCR);
	ax99100_i2c_write(ax, sdacr, I2C_CR);

	for (itr = 0; itr < I2C_NUM_ITER; itr++) {
		sclcr = ax99100_i2c_read(ax, I2C_SCLCR);

		if (!(sclcr & (I2C_SCLCR_RCVF | I2C_SCLCR_NACK)))
			break;

		/* Wait after the status check because sometimes the
		 * acknowledgment comes before the code gets to the status
		 * read point.
		 */
		usleep_range(70, 100);
	}

	if (itr == I2C_NUM_ITER) {
		/* Failed to recover the line. */
		if (sclcr & I2C_SCLCR_RCVF)
			return -EIO;

		/* Client is unavailable. */
		if (sclcr & I2C_SCLCR_NACK)
			return -ENXIO;
	}

	if (read_write == I2C_SMBUS_READ) {
		sdacr = ax99100_i2c_read(ax, I2C_CR);
		data->byte = I2C_CR_DATA(sdacr);
	}

	return 0;
}

static u32 ax99100_i2c_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WRITE_WORD_DATA;
}

static struct i2c_algorithm ax99100_i2c_algo = {
	.smbus_xfer = ax99100_i2c_xfer,
	.functionality = ax99100_i2c_func
};

static int ax99100_i2c_init(struct ax99100_data *ax)
{
	struct i2c_adapter *adapter = &ax->i2c.adapter;
	struct pci_dev *pdev = ax->pci.pdev;
	int ret;

	snprintf(adapter->name, sizeof(adapter->name), "ax99100_i2c");
	adapter->class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	adapter->algo = &ax99100_i2c_algo;
	adapter->algo_data = ax;
	adapter->dev.parent = &pdev->dev;

	ax99100_i2c_write(ax, I2C_SCLPR_400KHZ, I2C_SCLPR);

	ret = i2c_add_adapter(adapter);
	if (ret) {
		dev_err(&pdev->dev, "Couldn't register i2c controller\n");
		return ret;
	}

	dev_info(&pdev->dev, "i2c-%d 400KHz interface created\n", adapter->nr);

	return 0;
}

static void ax99100_i2c_clear(struct ax99100_data *ax)
{
	i2c_del_adapter(&ax->i2c.adapter);
}
#else
static int ax99100_i2c_init(struct ax99100_data *ax)
{
	return 0;
}

static void ax99100_i2c_clear(struct ax99100_data *ax) {}
#endif /* CONFIG_SPI_AX99100_I2C */

/*
 * ax99100 PCI general driver
 *
 * The PCI-SPI function is activated by setting the CHIP_MODE to 110/100.
 * In this case the PCI device provides three PCIe functions:
 * Functions 0-1: Unified/Multiprotocol Serial ports
 * Function  3: SPI controller
 * Each of the functions also provides an access to common interfaces like
 * i2c and GPIO by means of BAR5 MMIO registers. This driver supports the SPI
 * controller only. GPIO-related code is moved to the serial
 * ports driver (see 8250_ax99100 driver).
 */
static struct ax99100_data *ax99100_data_create(struct pci_dev *pdev)
{
	struct ax99100_data *ax;

	ax = devm_kzalloc(&pdev->dev, sizeof(*ax), GFP_KERNEL);
	if (!ax) {
		dev_err(&pdev->dev, "Memory allocation failed for data\n");
		return ERR_PTR(-ENOMEM);
	}

	ax->pci.pdev = pdev;

	return ax;
}

static int ax99100_pci_init(struct ax99100_data *ax)
{
	struct pci_dev *pdev = ax->pci.pdev;
	void __iomem * const *bars;
	int ret;

	ret = pcim_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable PCIe device\n");
		return ret;
	}

	pci_set_master(pdev);

	/* Map all PCIe memory regions provided by the device. */
	ret = pcim_iomap_regions(pdev, PCI_BAR_MASK, "ax99100_spi");
	if (ret) {
		dev_err(&pdev->dev, "Failed to request PCI regions\n");
		goto err_clear_master;
	}

	bars = pcim_iomap_table(pdev);

	ax->pci.sm = bars[0];
	ax->pci.dm = bars[1];
	ax->pci.im = bars[5];

	pci_set_drvdata(pdev, ax);

	ret = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI | PCI_IRQ_LEGACY);
	if (ret != 1) {
		dev_err(&pdev->dev, "Failed to alloc INTx IRQ vector\n");
		goto err_clear_drvdata;
	}

	ax->pci.irq = pci_irq_vector(pdev, 0);
	if (ax->pci.irq < 0) {
		dev_err(&pdev->dev, "Failed to get IRQ vector\n");
		goto err_free_irq_vectors;
	}

	return 0;

err_free_irq_vectors:
	pci_free_irq_vectors(pdev);

err_clear_drvdata:
	pci_set_drvdata(pdev, NULL);

err_clear_master:
	pci_clear_master(pdev);

	return ret;
}

static void ax99100_pci_clear(struct ax99100_data *ax)
{
	pci_free_irq_vectors(ax->pci.pdev);

	pci_set_drvdata(ax->pci.pdev, NULL);

	pci_clear_master(ax->pci.pdev);
}

static int ax99100_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct ax99100_data *ax;
	int ret;

	ax = ax99100_data_create(pdev);
	if (IS_ERR(ax))
		return PTR_ERR(ax);

	ret = ax99100_pci_init(ax);
	if (ret)
		return ret;

	ret = ax99100_spi_init(ax);
	if (ret)
		goto err_pci_clear;

	ret = ax99100_i2c_init(ax);
	if (ret)
		goto err_spi_clear;

	return 0;

err_spi_clear:
	ax99100_spi_clear(ax);

err_pci_clear:
	ax99100_pci_clear(ax);

	return ret;
}

static void ax99100_remove(struct pci_dev *pdev)
{
	struct ax99100_data *ax = pci_get_drvdata(pdev);

	ax99100_i2c_clear(ax);

	ax99100_spi_clear(ax);

	ax99100_pci_clear(ax);
}

static const struct pci_device_id ax99100_pci_id[] = {
	// { PCI_VENDOR_ID_ASIX, PCI_DEVICE_ID_ASIX_ax99100, 0xa000, 0x6000 },
	{ 0x125B, 0x9100, 0xa000, 0x6000 },	
	{ },
};
MODULE_DEVICE_TABLE(pci, ax99100_pci_id);

static struct pci_driver ax99100_driver = {
	.name           = "ax99100_spi",
	.id_table       = ax99100_pci_id,
	.probe          = ax99100_probe,
	.remove         = ax99100_remove
};
module_pci_driver(ax99100_driver);

MODULE_DESCRIPTION("Asix ax99100 PCI-SPI driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sergey Semin <fancer.lancer@gmail.com>");

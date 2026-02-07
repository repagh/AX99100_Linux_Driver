/* SPDX-License-Identifier: GPL-2.0
 *
 * PCI-I2C Driver for ax99100 multi-interface device
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/i2c.h>


#define PCI_CHIP_VER		0x44	/* 8 bits */
#define DRV_VERSION	"v2.3.0"
static char version[] =
KERN_INFO "ASIX AX99100 PCIe Bridg to I2C " DRV_VERSION
	"    http://www.asix.com.tw\n";

static char versionA[] =
KERN_INFO "ASIX AX99100A PCIe Bridg to I2C " DRV_VERSION
	"    http://www.asix.com.tw\n";


#define PCI_BAR_MASK		(BIT(0) | BIT(1) | BIT(5))
#define I2C_NUM_ITER		100

/*
 * Generic interface registers: i2c (MMIO - BAR5)
 */
#define I2C_CR			0x0C8
#define  I2C_CR_DATA(_x)	((_x) & GENMASK(7, 0))
#define  I2C_CR_MA(_x)		((_x) << 8 & GENMASK(23, 8))
#define  I2C_CR_MAF		BIT(24)
#define  I2C_CR_ADR(_x)		((_x) << 25 & GENMASK(30, 25))
#define  I2C_CR_RW_IND		BIT(31)
#define I2C_SCLPR		0x0CC
#define  I2C_SCLPR_100KHZ	(0xEE << 16 | 0x183)
#define  I2C_SCLPR_400KHZ	(0x3C << 16 | 0x61)
#define I2C_SCLCR		0x0D0
#define  I2C_SCLCR_ADR_LSB(_x)	((_x) & BIT(0))
#define  I2C_SCLCR_SBRT_ns(_x)	(((_x) / 16) << 1 & GENMASK(8, 1))
#define  I2C_SCLCR_SHSC_ns(_x)	(((_x) / 16) << 9 & GENMASK(24, 9))
#define  I2C_SCLCR_LHCEF	BIT(29)
#define  I2C_SCLCR_RCVF		BIT(30)
#define  I2C_SCLCR_NACK		BIT(31)
#define I2C_BFTR		0x0D4
#define  I2C_BFTR_BFT_ns(_x)	(((_x) / 16) & GENMASK(15, 0))

struct ax99100_pci {
	struct pci_dev *pdev;

	int irq;
	void __iomem *sm;
	void __iomem *dm;
	void __iomem *im;
};

struct ax99100_i2c {
	struct i2c_adapter adapter;
};

struct ax99100_data {
	struct ax99100_pci pci;

	struct ax99100_i2c i2c;
};

/*
 * ax99100 PCI-i2c interface driver
 *
 * ax99100 chips provides a functionally limited smbus-interface. It is made
 * mostly for EEPROM read/write operations and provides just SMBus
 * byte-read/byte-write/word-write data operations. Even though the operations
 * set is very limited it shall be enough to access some smbus-sensors,
 * i2c-muxes, GPIO-expanders.
 */
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
	adapter->class = I2C_CLASS_HWMON
#if LINUX_VERSION_CODE < KERNEL_VERSION(6,0,0)
	 | I2C_CLASS_SPD
#endif
	;
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



/*
 * ax99100 PCI general driver
 *
 * This driver supports the i2c interface only.
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
	ret = pcim_iomap_regions(pdev, PCI_BAR_MASK, "ax99100_i2c");
	if (ret) {
		dev_err(&pdev->dev, "Failed to request PCI regions\n");
		goto err_clear_master;
	}

	bars = pcim_iomap_table(pdev);

	ax->pci.sm = bars[0];
	ax->pci.dm = bars[1];
	ax->pci.im = bars[5];

	pci_set_drvdata(pdev, ax);

	ret = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI
#if LINUX_VERSION_CODE < KERNEL_VERSION(6,10,0)
		| PCI_IRQ_LEGACY
#else
		| PCI_IRQ_INTX
#endif
	);
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
	u8 data;

	ax = ax99100_data_create(pdev);
	if (IS_ERR(ax))
		return PTR_ERR(ax);

	ret = ax99100_pci_init(ax);
	if (ret)
		return ret;

	ret = ax99100_i2c_init(ax);
	if (ret)
		goto err_pci_clear;


	pci_read_config_byte(pdev, PCI_CHIP_VER, &data);
	data=data & 0x0f;
	// printk("In %s data=%d-----line %d\n", __FUNCTION__, data ,__LINE__);
	if (data==1){
		// printk("In %s data=%d-----line %d\n", __FUNCTION__, data ,__LINE__);
		printk(versionA);
	}else{
		printk(version);
	}
	return 0;

err_pci_clear:
	ax99100_pci_clear(ax);

	return ret;
}


static void ax99100_remove(struct pci_dev *pdev)
{
	struct ax99100_data *ax = pci_get_drvdata(pdev);

	ax99100_i2c_clear(ax);

	ax99100_pci_clear(ax);
}

static const struct pci_device_id ax99100_pci_id[] = {
    // { PCI_VENDOR_ID_ASIX, PCI_DEVICE_ID_ASIX_ax99100},
	{ PCI_DEVICE(0x125B, 0x9100) },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, ax99100_pci_id);

static struct pci_driver ax99100_driver = {
	.name           = "ax99100_i2c",
	.id_table       = ax99100_pci_id,
	.probe          = ax99100_probe,
	.remove         = ax99100_remove
};
module_pci_driver(ax99100_driver);

MODULE_DESCRIPTION("ASIX AX99100x PCI-I2C driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Tony Chung <tonychung@asix.com.tw>");

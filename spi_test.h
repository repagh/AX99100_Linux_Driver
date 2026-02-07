/* Flash OP code */
#define OPCODE_WREN		0x06
#define OPCODE_RDSR		0x05
	#define FLASH_STATUS_WREN	0x02
	#define FLASH_STATUS_ERASE	0x00
	#define FLASH_STATUS_PP		0x00
#define OPCODE_READ		0x03
#define OPCODE_SE		0x20
#define OPCODE_BE		0x52
#define OPCODE_CE		0x60
#define OPCODE_PP		0x02

#define WREN		0
#define RDSR		1
#define SE		2
#define BE		3
#define CE		4

/* Shift op-code */
#define OPCODE_STOF0		0
#define OPCODE_STOF1		8
#define OPCODE_STOF2		16
#define OPCODE_STOF3		24
#define OPCODE_STOF4		OPCODE_STOF0
#define OPCODE_STOF5		OPCODE_STOF1
#define OPCODE_STOF6		OPCODE_STOF2
#define OPCODE_STOF7		OPCODE_STOF3



#define TRUE	1
#define FALSE	0

#define TX	0
#define RX	1
#define TXRX	2

#define INTERRUPT	0
#define POLLING		1


#define	SUCCESS			0
#define ERROR_POLLING_TIMEOUT	-1
#define ERROR_SPI_TRANSCERIVER	-2

typedef unsigned char uchar;
typedef uchar BOOL;

/* 
 * 
 * Register (I/O mapped)
 * 
 */
#define REG_SPICMR		0x000
	#define SPICMR_SSP	(1 << 0)
	#define SPICMR_CPHA	(1 << 1)
	#define SPICMR_CPOL	(1 << 2)
	#define SPICMR_LSB	(1 << 3)
	#define SPICMR_SPIMEN	(1 << 4)
	#define SPICMR_ASS	(1 << 5)
	#define SPICMR_SWE	(1 << 6)
	#define SPICMR_SSOE	(1 << 7)
#define REG_SPICSS		0x001
#define REG_SPIBRR		0x004
#define REG_SPIDS		0x005
#define REG_SPIDT		0x006
#define REG_SDAOF		0x007
#define REG_STOF0		0x008
#define REG_STOF1		0x009
#define REG_STOF2		0x00A
#define REG_STOF3		0x00B
#define REG_STOF4		0x00C
#define REG_STOF5		0x00D
#define REG_STOF6		0x00E
#define REG_STOF7		0x00F
#define REG_SDFL0		0x010
#define REG_SDFL1		0x011
#define REG_SPISSOL		0x012
#define REG_SDCR		0x013
	#define INTERRUPT_ENABLE_MASK	0xC0
#define REG_SPIMISR		0x014
	#define INTERRUPT_MASK	0x003
	#define SPIMISR_STC	(1 << 0)
	#define SPIMISR_STERR	(1 << 1)

/* 
 * 
 * Register (MEM mapped)
 * 
 */

/* SPI Common Reg. */
#define REG_SWRST		0x238
	#define SW_RESET	(1 << 0)
/* TX DMA */
#define REG_TDMASAR0		0x080
#define REG_TDMASAR1		0x084
#define REG_TDMALR		0x088
#define REG_TDMASTAR		0x08C
	#define START_DMA	(1 << 0)
#define REG_TDMASTPR		0x090
#define REG_TDMASR		0x094
#define REG_TBNTS		0x098
/* RX DMA */
#define REG_RDMASAR0		0x100
#define REG_RDMASAR1		0x104
#define REG_RDMALR		0x108
#define REG_RDMASTAR		0x10C
#define REG_RDMASTPR		0x110
#define REG_RDMASR		0x114
#define REG_RBNTS		0x118

/* 
 * 
 * DMA setting
 * 
 */

#define	DMA_ABORT		(1 << 0)
#define	DMA_START		(1 << 0)
#define DMA_BUFFER_SZ		65535



#define PCI_SUBVEN_ID_AX99100_SPI 	0x6000

#define OFFSET_EEPORM		0x0C8

#define FL_BASE5                0x0005

#if defined(__i386__) && (defined(CONFIG_M386) || defined(CONFIG_M486))
#define _INLINE_ inline
#else
#define _INLINE_
#endif

#define DEFAULT99100_BAUD 115200

/* Device */
#define	DEV_NAME	"ax99100x"
#define	CLASS_NAME	"spidev"
#define NODE_NAME	"spi"


/* Netlink */
#define NETLINK_TEST 17
#define MAX_PAYLOAD_SIZE 1024
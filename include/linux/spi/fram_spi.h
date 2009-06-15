#define NEON270_FRAM_CLOCK	23
#define NEON270_FRAM_CS		24
#define NEON270_FRAM_SI		25
#define NEON270_FRAM_SO		26

/* Ramtron FM25L512 commands */
#define FRAM_WREN	0x06	/* Write enable */
#define FRAM_WRDI	0x04	/* Write disable */
#define FRAM_RDSR	0x05	/* Read status register */
#define FRAM_WRSR	0x01	/* Write status register */
#define FRAM_READ	0x03	/* Read data */
#define FRAM_WRITE	0x02	/* Write data */

/* Status register defn */
#define WPEN		7	/* equivalent to hardware write protection */

/* block protection modes */
#define BP_NONE		(0 << 2)	/* no protection */
#define BP_UPER_QTR	(1 << 2)	/* protect upper quarter (i.e.) 0xC000 - 0xFFFF */
#define BP_UPER_HF	(2 << 2)	/* protect upper half (i.e.) 0x8000 - 0xFFFF */
#define BP_ALL		(3 << 2)	/* protect all (i.e.) 0x0000 - 0xFFFF */

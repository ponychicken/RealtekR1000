/* This driver based on R1000 Linux Driver for Realtek controllers.
 * It's not supported by Realtek company, so use it for your own risk.
 * 2006 (c) Dmitri Arekhta (DaemonES@gmail.com)
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _REALTEKR1000_H_
#define _REALTEKR1000_H_

#include <IOKit/IOLib.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/network/IOEthernetController.h>
#include <IOKit/network/IOEthernetInterface.h>
#include <IOKit/network/IOGatedOutputQueue.h>
#include <IOKit/network/IOMbufMemoryCursor.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/IOFilterInterruptEventSource.h>

extern "C"
{
	#include <sys/kpi_mbuf.h>
	#include <architecture/i386/pio.h>>
}

#include "R1000Regs.h"
#include "mii.h"

#ifdef DEBUG
#define DbgPrint(args...) IOLog(args)
#else 
#define DbgPrint(args...)
#endif

#define RealtekR1000 rtl_r1000_nic_ext

enum
{
	MEDIUM_INDEX_10HD	= 0,
	MEDIUM_INDEX_10FD	= 1,
	MEDIUM_INDEX_100HD	= 2,
	MEDIUM_INDEX_100FD	= 3,
	MEDIUM_INDEX_1000HD = 4,
	MEDIUM_INDEX_1000FD = 5,
	MEDIUM_INDEX_AUTO	= 6,
	MEDIUM_INDEX_COUNT	= 7
};

enum 
{
    kActivationLevelNone = 0,  /* adapter shut off */
    kActivationLevelKDP,       /* adapter partially up to support KDP */
    kActivationLevelBSD        /* adapter fully up to support KDP and BSD */
};

class rtl_r1000_nic_ext : public IOEthernetController
{
	OSDeclareDefaultStructors(rtl_r1000_nic_ext)
public:
	virtual bool			init(OSDictionary *properties);
	virtual void			free();
	virtual bool			start(IOService *provider);
	virtual void			stop(IOService *provider);
	
	virtual IOReturn		enable(IONetworkInterface *netif);
    virtual IOReturn		disable(IONetworkInterface *netif);
	
    virtual UInt32			outputPacket(mbuf_t m, void *param);
    virtual void			getPacketBufferConstraints(IOPacketBufferConstraints *constraints) const;
    virtual IOOutputQueue	*createOutputQueue();
    virtual const OSString	*newVendorString() const;
    virtual const OSString	*newModelString() const;
    virtual IOReturn		selectMedium(const IONetworkMedium *medium);
    virtual bool			configureInterface(IONetworkInterface *netif);
    virtual bool			createWorkLoop();
    virtual IOWorkLoop		*getWorkLoop() const;
    virtual IOReturn		getHardwareAddress(IOEthernetAddress *addr);

    virtual IOReturn		setPromiscuousMode(bool enabled);
    virtual IOReturn		setMulticastMode(bool enabled);
    virtual IOReturn		setMulticastList(IOEthernetAddress *addrs, UInt32 count);

    virtual void			sendPacket(void *pkt, UInt32 pkt_len);
    virtual void			receivePacket(void * pkt, UInt32 *pkt_len, UInt32 timeout);

    virtual IOReturn		registerWithPolicyMaker(IOService *policyMaker);
    virtual IOReturn		setPowerState(unsigned long powerStateOrdinal, IOService *policyMaker);

	/*virtual IOReturn getMaxPacketSize(UInt32 *maxSize) const;	
	virtual IOReturn setMaxPacketSize(UInt32 maxSize); */
private:
	IOPCIDevice						*pciDev;
	IOWorkLoop						*workLoop;
	IOInterruptEventSource			*intSource;
    IOTimerEventSource				*timerSource;
    IONetworkStats					*netStats;
    IOEthernetStats					*etherStats;
	IOOutputQueue					*transmitQueue;
    IOEthernetInterface				*netif;
	OSDictionary					*mediumDict;
	const IONetworkMedium			*mediumTable[MEDIUM_INDEX_COUNT];

	UInt16							pioBase;
	IOMemoryMap						*mmioBase;
	bool							forcedPio;
	
	bool enabled;
	ushort vendorId, deviceId;
	bool linked;
	
	UInt32							activationLevel;
	bool							enabledForBSD;
	bool							enabledForKDP;

	int mcfg;
	int pcfg;
	int chipset;
	ulong expire_time;
	
	ulong mc_filter0, mc_filter1;
	
	unsigned long phy_link_down_cnt;
	unsigned long cur_rx;                   /* Index into the Rx descriptor buffer of next Rx pkt. */
	unsigned long cur_tx;                   /* Index into the Tx descriptor buffer of next Rx pkt. */
	unsigned long dirty_tx;

	uchar   drvinit_fail;

	struct	__mbuf				*Tx_skbuff[NUM_TX_DESC];	
	uchar						*Tx_dbuff[NUM_TX_DESC];
	IOBufferMemoryDescriptor	*Tx_skbuff_Md[NUM_TX_DESC];
	IOPhysicalAddress			Tx_skbuff_Dma[NUM_TX_DESC];
	
	uchar						*Rx_dbuff[NUM_RX_DESC];
	IOBufferMemoryDescriptor	*Rx_skbuff_Md[NUM_RX_DESC];
	IOPhysicalAddress			Rx_skbuff_Dma[NUM_RX_DESC];
	
	void *txdesc_space;
	struct	TxDesc	*TxDescArray;           /* Index of 256-alignment Tx Descriptor buffer */
	IOBufferMemoryDescriptor *tx_descMd;
	IOPhysicalAddress txdesc_phy_dma_addr;
	int sizeof_txdesc_space;

	void *rxdesc_space;
	struct	RxDesc	*RxDescArray;           /* Index of 256-alignment Rx Descriptor buffer */
	IOBufferMemoryDescriptor *rx_descMd;
	IOPhysicalAddress rxdesc_phy_dma_addr;
	int sizeof_rxdesc_space;
	
	int curr_mtu_size;
	int tx_pkt_len;
	int rx_pkt_len;

	int hw_rx_pkt_len;

	u16	speed;
	u8	duplex;
	u8	autoneg;
	
	static int max_interrupt_work;
	static int multicast_filter_limit;
	static const unsigned int ethernet_polynomial;

	inline void WriteMMIO8(ushort offset, uchar value)	 { (forcedPio) ? outb(pioBase + offset, value) : pciDev->ioWrite8(offset, value, mmioBase); }
	inline void WriteMMIO16(ushort offset, ushort value) { (forcedPio) ? outw(pioBase + offset, value) : pciDev->ioWrite16(offset, value, mmioBase); }
	inline void WriteMMIO32(ushort offset, ulong value)  { (forcedPio) ? outl(pioBase + offset, value) : pciDev->ioWrite32(offset, value, mmioBase); }
	
	inline uchar ReadMMIO8(ushort offset)   { return ((forcedPio) ? inb(pioBase + offset) : pciDev->ioRead8(offset, mmioBase)); }
	inline ushort ReadMMIO16(ushort offset) { return ((forcedPio) ? inw(pioBase + offset) : pciDev->ioRead16(offset, mmioBase)); }
	inline ulong ReadMMIO32(ushort offset)  { return ((forcedPio) ? inl(pioBase + offset) : pciDev->ioRead32(offset, mmioBase)); }
	
	void WriteGMII32(int RegAddr, int value );
	int ReadGMII32(int RegAddr);
	
	bool R1000InitBoard();
	bool R1000ProbeAndStartBoard();
	bool R1000StopBoard();
	
	bool R1000SetSpeedDuplex(ulong anar, ulong gbcr, ulong bmcr);
	bool R1000SetMedium(ushort speed, uchar duplex, uchar autoneg);
	
	bool increaseActivationLevel(UInt32 level);
	bool decreaseActivationLevel(UInt32 level);
	bool setActivationLevel(UInt32 level);
	
	void R1000HwPhyReset();
	void R1000HwPhyConfig();
	void R1000HwStart();
	
	ulong ether_crc(int length, unsigned char *data);
	
	bool AllocateDescriptorsMemory();
	void FreeDescriptorsMemory();
	
	bool R1000InitEventSources();
	bool R1000OpenAdapter();
	void R1000CloseAdapter();
	void R1000TxClear();
	
    void R1000Interrupt(OSObject * client, IOInterruptEventSource * src, int count);
	//bool R1000FilterInterrupt(OSObject *owner, IOFilterInterruptEventSource *src);
	void R1000RxInterrupt();
	void R1000TxInterrupt();
	void R1000TxTimeout(OSObject *owner, IOTimerEventSource * timer);
	
	bool OSAddNetworkMedium(ulong type, UInt32 bps, ulong index);
};

#endif
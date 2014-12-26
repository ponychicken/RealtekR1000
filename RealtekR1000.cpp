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

#include "RealtekR1000.h"

#define BaseClass IOEthernetController

#define RELEASE(x) do { if(x) { (x)->release(); (x) = 0; } } while(0)

OSDefineMetaClassAndStructors(rtl_r1000_nic_ext, IOEthernetController)

/* Some additional structs */
const static struct RtlChipInfo 
{
	const char *name;
	u8 mcfg;		 /* depend on documents of Realtek */
	u32 RxConfigMask; 	/* should clear the bits supported by this chip */
} rtl_chip_info[] = 
{
	{ "RTL8169",			MCFG_METHOD_1,	0xff7e1880 },
	{ "RTL8169S/8110S",		MCFG_METHOD_2,  0xff7e1880 },
	{ "RTL8169S/8110S",		MCFG_METHOD_3,  0xff7e1880 },
	{ "RTL8169SB/8110SB",	MCFG_METHOD_4,  0xff7e1880 },
	{ "RTL8169SC/8110SC",	MCFG_METHOD_5,  0xff7e1880 },
	{ "RTL8168B/8111B",		MCFG_METHOD_11, 0xff7e1880 },
	{ "RTL8168B/8111B",		MCFG_METHOD_12, 0xff7e1880 },
	{ "RTL8101E",			MCFG_METHOD_13, 0xff7e1880 },
	{ "RTL8100E",			MCFG_METHOD_14, 0xff7e1880 },
	{ "RTL8100E",			MCFG_METHOD_15, 0xff7e1880 },
	{ 0 }
};

/* Maximum events (Rx packets, etc.) to handle at each interrupt. */
int RealtekR1000::max_interrupt_work = 20;

/* Maximum number of multicast addresses to filter (vs. Rx-all-multicast).
   The RTL chips use a 64 element hash table based on the Ethernet CRC.  */
int RealtekR1000::multicast_filter_limit = 32;

const unsigned int RealtekR1000::ethernet_polynomial = 0x04c11db7U;

static const u16 r1000_intr_mask = LinkChg | RxOverflow | RxFIFOOver | TxErr | TxOK | RxErr | RxOK ;
static const unsigned int r1000_rx_config = (RX_FIFO_THRESH << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift) | 0x0000000E;

/*
 * Initialization of driver instance,
 * i.e. resources allocation and so on.
*/
bool RealtekR1000::init(OSDictionary *properties)
{
	DbgPrint("[RealtekR1000] RealtekR1000::init(OSDictionary *properties)\n");
	if (BaseClass::init(properties) == false) return false;
	
	pciDev = NULL;
	mmioBase = NULL;
	workLoop = NULL;
	intSource = NULL;
    timerSource = NULL;
    netStats = NULL;
    etherStats = NULL;
	transmitQueue = NULL;
    netif = NULL;
	enabled = false;
	forcedPio = false;
	enabledForKDP = enabledForBSD = false;	
	
	return true;
}

/*
 * Calling before destroing driver instance.
 * Frees all allocated resources.
*/
void RealtekR1000::free()
{
	DbgPrint("[RealtekR1000] RealtekR1000::free()");
	
	//free resource of base instance
	if (intSource && workLoop)
	{
		//Detaching interrupt source from work loop
		workLoop->removeEventSource(intSource);
	}
  
	RELEASE(netif);
    RELEASE(intSource);
    RELEASE(timerSource);
    RELEASE(mmioBase);
    RELEASE(pciDev);
    RELEASE(workLoop);
	
	FreeDescriptorsMemory();
	
	BaseClass::free();
} 

/*
 * Starting driver.
*/
bool RealtekR1000::start(IOService *provider)
{
	DbgPrint("[RealtekR1000] RealtekR1000::start(IOService *provider)\n");
	pciDev = OSDynamicCast(IOPCIDevice, provider);
	if (!pciDev)
	{
		DbgPrint("[RealtekR1000] failed to cast provider\n");
		return false;
	}
	if (BaseClass::start(pciDev) == false) return false;
	pciDev->retain();	
	pciDev->open(this);
	
	chipset = 5; //Assuming RTL8168/8111 by default
	
	//Adding Mac OS X PHY's
	mediumDict = OSDictionary::withCapacity(MEDIUM_INDEX_COUNT + 1);

	OSAddNetworkMedium(kIOMediumEthernetAuto, 0, MEDIUM_INDEX_AUTO);	
	OSAddNetworkMedium(kIOMediumEthernet10BaseT | kIOMediumOptionHalfDuplex, 10 * MBit, MEDIUM_INDEX_10HD);
	OSAddNetworkMedium(kIOMediumEthernet10BaseT | kIOMediumOptionFullDuplex, 10 * MBit, MEDIUM_INDEX_10FD);
	OSAddNetworkMedium(kIOMediumEthernet100BaseTX | kIOMediumOptionHalfDuplex, 100 * MBit, MEDIUM_INDEX_100HD);
	OSAddNetworkMedium(kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex, 100 * MBit, MEDIUM_INDEX_100FD);	
	OSAddNetworkMedium(kIOMediumEthernet1000BaseTX | kIOMediumOptionHalfDuplex, 1000 * MBit, MEDIUM_INDEX_1000HD);
	OSAddNetworkMedium(kIOMediumEthernet1000BaseTX | kIOMediumOptionFullDuplex, 1000 * MBit, MEDIUM_INDEX_1000FD);
		
	if (!publishMediumDictionary(mediumDict)) return false;
	
	if (!R1000ProbeAndStartBoard()) return false;
	
	if (!AllocateDescriptorsMemory()) return false;
	
	if (!R1000InitEventSources()) return false;
	
	//Attaching dynamic link layer
	if (!attachInterface(reinterpret_cast<IONetworkInterface **>(&netif)), false)
	{
		DbgPrint("[RealtekR1000] Failed to attach data link layer\n");
		return false;
	}
	
	netif->registerService();
	pciDev->close(this);
	
	return true;
}

/*
 * Stopping driver.
*/
void RealtekR1000::stop(IOService *provider)
{
	DbgPrint("[RealtekR1000] RealtekR1000::stop(IOService *provider)\n");
	detachInterface(netif);
	R1000StopBoard();
	BaseClass::stop(provider);
}

bool RealtekR1000::OSAddNetworkMedium(ulong type, UInt32 bps, ulong index)
{
	IONetworkMedium *medium;
	
	medium = IONetworkMedium::medium( type, bps, 0, index );
	if (!medium) 
	{
		DbgPrint("[RealtekR1000] Couldn't allocate medium\n");		
		return false;
	}
	if (!IONetworkMedium::addMedium(mediumDict, medium)) 
	{
		DbgPrint("[RealtekR1000] Couldn't add medium\n");
		return false;
	}
	mediumTable[index] = medium;
	return true;
}

bool RealtekR1000::increaseActivationLevel(UInt32 level)
{
	bool ret = false;

	switch (level)
	{
	case kActivationLevelKDP:
		if (!pciDev) break;
		pciDev->open(this);
		
		// PHY medium selection.
		const IONetworkMedium *medium = getSelectedMedium();
		if (!medium)
		{
			DbgPrint("[RealtekR1000] Selected medium is NULL, forcing to autonegotiation\n");
			medium = mediumTable[MEDIUM_INDEX_AUTO];
		}
		else
		{
			DbgPrint("[RealtekR1000] Selected medium index %d", medium->getIndex());
		}
		
		selectMedium(medium);
		timerSource->setTimeoutMS(TX_TIMEOUT);
		ret = true;
		break;
	case kActivationLevelBSD:
		if (!R1000OpenAdapter()) break;
		transmitQueue->setCapacity(kTransmitQueueCapacity);
		transmitQueue->start();
		
		ret = true;
		break;
	}
	
	return ret;
}

bool RealtekR1000::decreaseActivationLevel(UInt32 level)
{
	switch (level)
	{
	case kActivationLevelKDP:
		timerSource->cancelTimeout();
		
		if (pciDev) pciDev->close(this);
		break;
	case kActivationLevelBSD:
		transmitQueue->stop();
	
		transmitQueue->setCapacity(0);
		transmitQueue->flush();
		R1000CloseAdapter();
		break;
	}
	
	return true;
}

bool RealtekR1000::setActivationLevel(UInt32 level)
{
    bool success = false;

	DbgPrint("[RealtekR1000] setActivationLevel %d\n", level);

    if (activationLevel == level) return true;

    for ( ; activationLevel > level; activationLevel--) 
    {
        if ((success = decreaseActivationLevel(activationLevel)) == false)
            break;
    }

    for ( ; activationLevel < level; activationLevel++ ) 
    {
        if ((success = increaseActivationLevel(activationLevel+1)) == false)
            break;
    }

    return success;
}

/*
 * A request from an interface client to enable the controller.
*/
IOReturn RealtekR1000::enable(IONetworkInterface *netif)
{
	DbgPrint("[RealtekR1000] RealtekR1000::enable(IONetworkInterface *netif)\n");
	
	if (enabledForBSD) return kIOReturnSuccess;
	
	enabledForBSD = setActivationLevel(kActivationLevelBSD);
	if (enabledForBSD)
	{
		return kIOReturnSuccess;
	}
	else return kIOReturnIOError;
}

/*
 * A request from an interface client to disable the controller.
*/
IOReturn RealtekR1000::disable(IONetworkInterface *netif)
{
	enabledForBSD = false;
	
	setActivationLevel(enabledForKDP ? kActivationLevelKDP : kActivationLevelNone);
	
	return kIOReturnSuccess;
}

/*
 * Transmits an output packet.
 * packet - an mbuf chain containing the output packet to be sent on the network.
 * param - a parameter provided by the caller.
*/
UInt32 RealtekR1000::outputPacket(mbuf_t m, void *param)
{
	DbgPrint("[RealtekR1000] RedaltekR1000::outputPacket(mbuf_t m, void *param)\n");
	
	int entry = cur_tx % NUM_TX_DESC;
	int buf_len = 60;
	
	
	
	if ((OSSwapLittleToHostInt32(TxDescArray[entry].status) & OWNbit) == 0)
	{	
		if (mbuf_pkthdr_len(m) <= tx_pkt_len) buf_len = mbuf_pkthdr_len(m);
		else
		{
			DbgPrint("[RealtekR1000] Tx Packet size is too big, droping\n");
			freePacket(m);
			return kIOReturnOutputDropped;
		}
		
		Tx_skbuff[entry] = m;																					
		TxDescArray[entry].buf_addr = OSSwapHostToLittleInt32(Tx_skbuff_Dma[entry]);
		
		uchar *data_ptr = Tx_dbuff[entry];
		ulong pkt_snd_len = 0;
		mbuf_t cur_buf = m;
		
		do
		{
			if (mbuf_data(cur_buf))	bcopy(mbuf_data(cur_buf), data_ptr, mbuf_len(cur_buf));
			data_ptr += mbuf_len(cur_buf);
			pkt_snd_len += mbuf_len(cur_buf);
		}
		while(((cur_buf = mbuf_next(cur_buf)) != NULL) && ((pkt_snd_len + mbuf_len(cur_buf)) <= buf_len));
		buf_len = pkt_snd_len;
		
		
		if (entry != (NUM_TX_DESC - 1))
		{
			TxDescArray[entry].status = OSSwapHostToLittleInt32((OWNbit | FSbit | LSbit) | buf_len);
		}
		else
		{
			TxDescArray[entry].status = OSSwapHostToLittleInt32((OWNbit | EORbit | FSbit | LSbit) | buf_len);
		}
		
		WriteMMIO8 ( TxPoll, 0x40);		//set polling bit
		cur_tx++;
		
		DbgPrint("[RealtekR1000] mbuf_len %d, packet_len %d\n", mbuf_len(m), mbuf_pkthdr_len(m));
		DbgPrint("[RealtekR1000] cur_tx %d, dirty_tx %d, buf_len %d\n", cur_tx, dirty_tx, buf_len, mbuf_data(m), mbuf_data_to_physical(mbuf_data(m)));
	}
	else 
	{
		DbgPrint("[RealtekR1000] TX_RING_IS_FULL stalling\n");
		return kIOReturnOutputStall;
	}

	/*if ((cur_tx - NUM_TX_DESC) == dirty_tx)
	{
		transmitQueue->stop();
	}
	else
	{
		transmitQueue->start();
	}*/
	return kIOReturnOutputSuccess;
}

void RealtekR1000::getPacketBufferConstraints(IOPacketBufferConstraints *constraints) const
{
	DbgPrint("[RealtekR1000] RealtekR1000::getPacketBufferConstraints(IOPacketBufferConstraints *constraints) const\n");
	constraints->alignStart = kIOPacketBufferAlign4;
	constraints->alignLength = kIOPacketBufferAlign4;
}

IOOutputQueue *RealtekR1000::createOutputQueue()
{
	DbgPrint("[RealtekR1000] RealtekR1000::createOutputQueue()\n");
	//Sharing one event source with transmith/receive handles
	return IOGatedOutputQueue::withTarget(this, getWorkLoop());
}

/*
 * Returns a string describing the vendor of the network controller. The caller is responsible for releasing the string object returned.
*/
const OSString *RealtekR1000::newVendorString() const
{
	DbgPrint("[RealtekR1000] RealtekR1000::newVendorString() const\n");
	return OSString::withCString("Realtek");
}

/*
 * Returns a string describing the model of the network controller. The caller is responsible for releasing the string object returned.
*/
const OSString *RealtekR1000::newModelString() const
{
	DbgPrint("[RealtekR1000] RealtekR1000::newModelString() const\n");
	return OSString::withCString(rtl_chip_info[chipset].name);
}

/*
 * A client request to change the medium selection.
 * This method is called when a client issues a command for the controller to change its 
 * current medium selection. The implementation must call setSelectedMedium() after the change 
 * has occurred. This method call is synchronized by the workloop's gate.
*/
IOReturn RealtekR1000::selectMedium(const IONetworkMedium *medium)
{
	DbgPrint("[RealtekR1000] RealtekR1000::selectMedium(const IONetworkMedium *medium)\n");
	DbgPrint("[RealtekR1000] index %d\n", medium->getIndex());

	if (medium) 
	{
		switch (medium->getIndex())
		{
		case MEDIUM_INDEX_AUTO:
			R1000SetMedium(SPEED_100, DUPLEX_FULL, AUTONEG_ENABLE);
			break;
		case MEDIUM_INDEX_10HD:
			R1000SetMedium(SPEED_10, DUPLEX_HALF, AUTONEG_DISABLE);
			break;
		case MEDIUM_INDEX_10FD:
			R1000SetMedium(SPEED_10, DUPLEX_FULL, AUTONEG_DISABLE);
			break;
		case MEDIUM_INDEX_100HD:
			R1000SetMedium(SPEED_100, DUPLEX_HALF, AUTONEG_DISABLE);
			break;
		case MEDIUM_INDEX_100FD:
			R1000SetMedium(SPEED_100, DUPLEX_FULL, AUTONEG_DISABLE);
			break;
		case MEDIUM_INDEX_1000HD:
			R1000SetMedium(SPEED_1000, DUPLEX_HALF, AUTONEG_DISABLE);
			break;
		case MEDIUM_INDEX_1000FD:
			R1000SetMedium(SPEED_1000, DUPLEX_FULL, AUTONEG_DISABLE);
			break;
		}
		setCurrentMedium(medium);
	}
	else
	{
		DbgPrint("[RealtekR1000] Selected medium is NULL\n");
	}
	
	if (ReadMMIO8(PHYstatus) & LinkStatus)
	{
		if (ReadMMIO8(PHYstatus) & _1000Mbps) setLinkStatus(kIONetworkLinkActive | kIONetworkLinkValid, getSelectedMedium(), 1000 * MBit, NULL);
		else if(ReadMMIO8(PHYstatus) & _100Mbps) setLinkStatus(kIONetworkLinkActive | kIONetworkLinkValid, getSelectedMedium(), 100 * MBit, NULL);
		else if(ReadMMIO8(PHYstatus) & _10Mbps) setLinkStatus(kIONetworkLinkActive | kIONetworkLinkValid, getSelectedMedium(), 10 * MBit, NULL);
	}
	else
	{	
		setLinkStatus(kIONetworkLinkValid, NULL, 0, NULL);
	}

	return kIOReturnSuccess;
}

/*
 * Configures a newly created network interface object.
 * This method configures an interface object that was created by createInterface(). 
 * Subclasses can override this method to customize and examine the interface object that will be 
 * attached to the controller as a client.
*/
bool RealtekR1000::configureInterface(IONetworkInterface *netif)
{
	DbgPrint("[RealtekR1000] RealtekR1000::configureInterface(IONetworkInterface *interface)\n");
	IONetworkData * data;

    if (!BaseClass::configureInterface(netif)) return false;
	
    // Get the generic network statistics structure.
    data = netif->getParameter( kIONetworkStatsKey );
    if ( !data || !(netStats = (IONetworkStats *) data->getBuffer()) ) 
    {
        return false;
    }

    // Get the Ethernet statistics structure.
    data = netif->getParameter( kIOEthernetStatsKey );
    if ( !data || !(etherStats = (IOEthernetStats *) data->getBuffer()) ) 
    {
        return false;
    }

    return true;
}

/*
 * Method called by IONetworkController prior to the initial getWorkLoop() call.
*/
bool RealtekR1000::createWorkLoop()
{
	DbgPrint("[RealtekR1000] RealtekR1000::createWorkLoop()\n");
	workLoop = IOWorkLoop::workLoop();
	if (workLoop) return true;
	else return false;
}

IOWorkLoop *RealtekR1000::getWorkLoop() const
{
	DbgPrint("[RealtekR1000] RealtekR1000::getWorkLoop()\n");
	return workLoop;
}

/*
 * Gets the Ethernet controller's permanent station address.
 * Ethernet drivers must implement this method, by reading the address from hardware and writing 
 * it to the buffer provided. This method is called from the workloop context.
*/
IOReturn RealtekR1000::getHardwareAddress(IOEthernetAddress *addr)
{
	DbgPrint("[RealtekR1000] RealtekR1000::getHardwareAddress(IOEthernetAddress *addr)\n");
	uchar bytes[MAC_ADDR_LEN];
	
	for (uchar i = 0; i < MAC_ADDR_LEN ; i++)
	{
		bytes[i] = ReadMMIO8(MAC0 + i);
	}
	
	addr->bytes[0] = bytes[0];
    addr->bytes[1] = bytes[1];
    addr->bytes[2] = bytes[2];
    addr->bytes[3] = bytes[3];
    addr->bytes[4] = bytes[4];
    addr->bytes[5] = bytes[5];
	return kIOReturnSuccess;
}

/*
 * Enables or disables promiscuous mode.
 * Called by enablePacketFilter() or disablePacketFilter() when there is a change 
 * in the activation state of the promiscuous filter identified by kIOPacketFilterPromiscuous. 
 * This method is called from the workloop context.
*/
IOReturn RealtekR1000::setPromiscuousMode(bool enabled)
{
	DbgPrint("[RealtekR1000] RealtekR1000::setPromiscuousMode(bool enabled)\n");
	
	ulong rx_mode;
	ulong mc_filter[2];
	
	if (enabled)
	{
		//Accept all multicasts
		mc_filter[0] = mc_filter[1] = 0xffffffff;
		
		rx_mode = r1000_rx_config | AcceptBroadcast | AcceptMulticast | AcceptMyPhys | AcceptAllPhys |
					(ReadMMIO32(RxConfig) & rtl_chip_info[chipset].RxConfigMask);
	}
	else
	{
		//Restoring old multicast filter
		mc_filter[0] = mc_filter0;
		mc_filter[1] = mc_filter1;
		
		rx_mode = r1000_rx_config | 
					AcceptBroadcast | AcceptMulticast | AcceptMyPhys | 
					(ReadMMIO32(RxConfig) & rtl_chip_info[chipset].RxConfigMask);
	}
	
	WriteMMIO32(RxConfig, rx_mode);
	if ((mcfg == MCFG_METHOD_11) || (mcfg == MCFG_METHOD_12) ||
	    (mcfg == MCFG_METHOD_13) || (mcfg == MCFG_METHOD_14) ||
	    (mcfg == MCFG_METHOD_15)) 
	{
		WriteMMIO32(MAR0 + 0, 0xffffffff);
		WriteMMIO32(MAR0 + 4, 0xffffffff);
	}
	else
	{
		WriteMMIO32(MAR0 + 0, mc_filter[0]);
		WriteMMIO32(MAR0 + 4, mc_filter[1]);
	}
	
	return kIOReturnSuccess;
}

/*
 * Enables or disables multicast mode.
 * Called by enablePacketFilter() or disablePacketFilter() when there is a 
 * change in the activation state of the multicast filter identified by kIOPacketFilterMulticast. 
 * This method is called from the workloop context.
*/
IOReturn RealtekR1000::setMulticastMode(bool enabled)
{
	DbgPrint("[RealtekR1000] RealtekR1000::setMulticastMode(bool enabled)\n");
		
	ulong rx_mode;
	if (enabled)
	{
		rx_mode = r1000_rx_config | 
					AcceptBroadcast | AcceptMulticast | AcceptMyPhys | 
					(ReadMMIO32(RxConfig) & rtl_chip_info[chipset].RxConfigMask);
	}
	else
	{
		rx_mode = (r1000_rx_config | (ReadMMIO32(RxConfig) & rtl_chip_info[chipset].RxConfigMask)) &
					~AcceptMulticast;	
	}

	WriteMMIO32(RxConfig, rx_mode);
	return kIOReturnSuccess;
}

/* Sets the list of multicast addresses a multicast filter should use to match 
 * against the destination address of an incoming frame.
 * This method sets the list of multicast addresses that the multicast filter should use 
 * to match against the destination address of an incoming frame. The frame should be 
 * accepted when a match occurs. Called when the multicast group membership of an interface object is changed. 
 * Drivers that support kIOPacketFilterMulticast should override this method and update the 
 * hardware multicast filter using the list of Ethernet addresses provided. Perfect multicast filtering 
 * is preferred if supported by the hardware, in order to reduce the number of unwanted packets received. 
 * If the number of multicast addresses in the list exceeds what the hardware is capable of supporting, 
 * or if perfect filtering is not supported, then ideally the hardware should be programmed to perform 
 * imperfect filtering, through some form of hash filtering mechanism. Only as a last resort should the driver 
 * enable reception of all multicast packets to satisfy this request. This method is called from the workloop 
 * context, and only if the driver reports kIOPacketFilterMulticast support in getPacketFilters().
*/
IOReturn RealtekR1000::setMulticastList(IOEthernetAddress *addrs, UInt32 count)
{
	DbgPrint("[RealtekR1000] RealtekR1000::setMulticastList(IOEthernetAddress *addrs, UInt32 count)\n");
	//DbgPrint("[RealtekR1000] Forcing method!\n");
	//return kIOReturnSuccess;

	//It seems, that RTL8168 family uses the same filter hashing method, as VIA Rhine,
	//so this part of code taken from ViaRhine Mac OS X driver by Yang Wu
/*	
#ifdef DEBUG
	for (int i = 0; i < kIOEthernetAddressSize; ++i )
	{
		DbgPrint("%x:", addrs->bytes[i]);
	}
	DbgPrint(", count = %d. \n", count );
#endif
*/
	ulong mc_filter[2];
	
	if (count > multicast_filter_limit)
	{
		DbgPrint("[RealtekR1000] Break multicast filter limit, accept all!");
		mc_filter[0] = mc_filter[1] = 0xffffffff;
	}
	else 
	{
		for ( UInt32 i = 0; i < count; i++, addrs++ )
		{
			int bit = 0;
			bit = ether_crc(6, reinterpret_cast<uchar *>(addrs)) >> 26;
			DbgPrint("[RealtekR1000] Multicast bits after crc calc: 0x%8.8x.\n", bit);
			if (bit < 32) mc_filter[0] |= (1 << bit);
			else mc_filter[1] |= (1 << (bit - 32));
		}
	}

	if ((mcfg == MCFG_METHOD_11) || (mcfg == MCFG_METHOD_12) ||
	    (mcfg == MCFG_METHOD_13) || (mcfg == MCFG_METHOD_14) ||
	    (mcfg == MCFG_METHOD_15)) 
	{
		WriteMMIO32(MAR0 + 0, 0xffffffff);
		WriteMMIO32(MAR0 + 4, 0xffffffff);
	}
	else
	{
		WriteMMIO32(MAR0 + 0, mc_filter[0]);
		WriteMMIO32(MAR0 + 4, mc_filter[1]);
	}
	
	return kIOReturnSuccess;
}

/*
 * Debugger polled-mode transmit handler.
 * This method must be implemented by a driver that supports kernel debugging.
 * pkt - pointer to a transmit buffer containing the packet to be sent on the network.
 * pktSize - the size of the transmit buffer in bytes.
*/
void RealtekR1000::sendPacket(void * pkt, UInt32 pkt_len)
{
	DbgPrint("[RealtekR1000] RealtekR1000::sendPacket(void * pkt, UInt32 pkt_len)\n");
}

/*
 * Debugger polled-mode receive handler.
 * This method must be implemented by a driver that supports kernel debugging.
 * pkt - address of a receive buffer where the received packet should be stored. This buffer has room for 1518 bytes.
 * pktSize - address where the number of bytes received must be recorded. Set this to zero if no packets were received during the timeout interval.
 * timeout - the maximum amount of time in milliseconds to poll for a packet to arrive before this method must return.
*/
void RealtekR1000::receivePacket(void *pkt, UInt32 *pkt_len, UInt32 timeout)
{
	DbgPrint("[RealtekR1000] RealtekR1000::receivePacket(void *pkt, UInt32 *pkt_len, UInt32 timeout)\n");
}

/*
 * Implements the framework for a generic network controller.
*/
IOReturn RealtekR1000::registerWithPolicyMaker(IOService *policyMaker)
{
	DbgPrint("[RealtekR1000] RealtekR1000::registerWithPolicyMaker(IOService *policyMaker)\n");
	
	//Grabed from ViaRhine
    enum 
	{
        kPowerStateOff = 0,
        kPowerStateOn,
        kPowerStateCount
    };

    static IOPMPowerState powerStateArray[ kPowerStateCount ] =
    {
        { 1,0,0,0,0,0,0,0,0,0,0,0 },
        { 1,IOPMDeviceUsable,IOPMPowerOn,IOPMPowerOn,0,0,0,0,0,0,0,0 }
    };

    IOReturn ret;

    ret = policyMaker->registerPowerDriver( this, powerStateArray,
                                            kPowerStateCount );
    
    return ret;
}

IOReturn RealtekR1000::setPowerState(unsigned long powerStateOrdinal, IOService *policyMaker)
{
	DbgPrint("[RealtekR1000] RealtekR1000::setPowerState(unsigned long powerStateOrdinal, IOService *policyMaker)\n");
	//TO-DO: Add power state support
	return IOPMAckImplied;
}

/*IOReturn RealtekR1000::getMaxPacketSize(UInt32 *maxSize) const
{
	return kIOReturnUnsupported;
	*maxSize = MAX_JUMBO_FRAME_MTU + ETH_HDR_LEN + FCS_LEN;
	
	return kIOReturnSuccess;
}*/

/*IOReturn RealtekR1000::setMaxPacketSize(UInt32 maxSize)
{
	DbgPrint("[RealtekR1000] RealtekR1000::setMaxPacketSize(UInt32 maxSize) Packet size %d", maxSize);
	
	maxSize = maxSize - ETH_HDR_LEN - FCS_LEN;
	if (maxSize > MAX_JUMBO_FRAME_MTU)
	{
		DbgPrint("[RealtekR1000] New MTU is too high!\n");
		return kIOReturnUnsupported;
	}
	
	curr_mtu_size = maxSize;
	tx_pkt_len = maxSize + ETH_HDR_LEN;
	rx_pkt_len = maxSize + ETH_HDR_LEN;
	hw_rx_pkt_len = rx_pkt_len + 8;
	
	WriteMMIO8 ( Cfg9346, Cfg9346_Unlock);
	WriteMMIO16	(RxMaxSize, static_cast<unsigned short>(hw_rx_pkt_len));
	WriteMMIO8 ( Cfg9346, Cfg9346_Lock);
	
	R1000CloseAdapter();
	R1000OpenAdapter();
	
	return kIOReturnSuccess;
}*/


/*
 *  Private methods
 */

//=================================================================
//	PHYAR
//	bit		Symbol
//	31		Flag
//	30-21	reserved
//	20-16	5-bit GMII/MII register address
//	15-0	16-bit GMII/MII register data
//=================================================================
void RealtekR1000::WriteGMII32(int RegAddr, int value )
{
	int	i;

	WriteMMIO32(PHYAR, 0x80000000 | (RegAddr & 0xFF) << 16 | value);
	//udelay(1000); //Microseconds ?
	IODelay(1000);

	for( i = 2000; i > 0 ; i--)
	{
		// Check if the RTL8169 has completed writing to the specified MII register
		if (!(ReadMMIO32(PHYAR) & 0x80000000))
		{
			break;
		}
		else
		{
			//udelay(100);
			IODelay(100);
		}// end of if( ! (RTL_R32(PHYAR)&0x80000000) )
	}// end of for() loop
}

int RealtekR1000::ReadGMII32(int RegAddr)
{
	int i, value = -1;

	WriteMMIO32( PHYAR, 0x0 | (RegAddr&0xFF)<<16);
	//udelay(1000);
	IODelay(1000);

	for( i = 2000; i > 0 ; i--){
		// Check if the RTL8169 has completed retrieving data from the specified MII register
		if( ReadMMIO32(PHYAR) & 0x80000000 )
		{
			value = static_cast<int>((ReadMMIO32(PHYAR) & 0xFFFF));
			break;
		}
		else
		{
			//udelay(100);
			IODelay(100);
		}// end of if( RTL_R32(PHYAR) & 0x80000000 )
	}// end of for() loop
	return value;
}

bool RealtekR1000::R1000InitBoard()
{
	pciDev->setBusMasterEnable(true);
	pciDev->setIOEnable(true);
	pciDev->setMemoryEnable(true);	
	
	vendorId = pciDev->configRead16(0);
	deviceId = pciDev->configRead16(2);
	pciDev->enablePCIPowerManagement();
	
	IOSleep(10);
	
	pioBase = pciDev->configRead16(kIOPCIConfigBaseAddress0) & 0xFFFC;
	DbgPrint("[RealtekR1000] mmio/pio base 0x%04x\n", pioBase);
	mmioBase = pciDev->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0);
	
	if (!mmioBase) 
	{
		DbgPrint("[RealtekR1000] Couldn't setup memory io\n");
		DbgPrint("[RealtekR1000] Forcing to port IO\n");
		pciDev->setMemoryEnable(false);
		pciDev->setIOEnable(true);
		forcedPio = true;
	}
	else DbgPrint("[RealtekR1000] Memory mapped at virt addr 0x%x, phys addr 0x%x\n", mmioBase->getVirtualAddress(),
						mmioBase->getPhysicalAddress());
						
						
		// Soft reset the chip.
	WriteMMIO8 ( ChipCmd, CmdReset);

	// Check that the chip has finished the reset.
	for (int i = 1000; i > 0; i--)
	{
		if ((ReadMMIO8(ChipCmd) & CmdReset) == 0)
		{
			break;
		}
		else
		{
			IODelay(10);
		}
	}
	
	// identify config method
	{
		unsigned long val32 = (ReadMMIO32(TxConfig) & 0x7c800000);

		if (val32 == 0x38800000) mcfg = MCFG_METHOD_15;
		else if (val32 == 0x30800000) mcfg = MCFG_METHOD_14;
		else if (val32 == 0x34000000) mcfg = MCFG_METHOD_13;
		else if (val32 == 0x38000000) mcfg = MCFG_METHOD_12;
		else if (val32 == 0x30000000) mcfg = MCFG_METHOD_11;
		else if (val32 == 0x18000000) mcfg = MCFG_METHOD_5;
		else if (val32 == 0x10000000) mcfg = MCFG_METHOD_4;
		else if (val32 == 0x04000000) mcfg = MCFG_METHOD_3;
		else if (val32 == 0x00800000) mcfg = MCFG_METHOD_2;
		else if (val32 == 0x00000000) mcfg = MCFG_METHOD_1;
		else mcfg = MCFG_METHOD_1;
	}
	{
		uchar val8 = static_cast<uchar>(ReadGMII32(3)&0x000f);
		if (val8 == 0x00)
		{
			pcfg = PCFG_METHOD_1;
		}
		else if (val8 == 0x01)
		{
			pcfg = PCFG_METHOD_2;
		}
		else if (val8 == 0x02) 
		{
			pcfg = PCFG_METHOD_3;
		}
		else
		{
			pcfg = PCFG_METHOD_3;
		}
	}

	DbgPrint("[RealtekR1000] mcfg %d, pcfg %d\n", mcfg, pcfg);
	
	for(int i = sizeof(rtl_chip_info)/sizeof(RtlChipInfo) - 1; i >= 0; --i)
		if (rtl_chip_info[i].mcfg == mcfg) chipset = i;
		
	DbgPrint("[RealtekR1000] Chip %s detected\n", rtl_chip_info[chipset].name);
	return true;
}

bool RealtekR1000::R1000ProbeAndStartBoard()
{
	if (!R1000InitBoard()) return false;

#ifdef DEBUG
	// Get MAC address //
	DbgPrint("[RealtekR1000] MAC address: ");
	for (uchar i = 0; i < MAC_ADDR_LEN ; i++)
	{
		DbgPrint("%02x ", ReadMMIO8(MAC0 + i));
	}
	DbgPrint("\n");
#endif

	curr_mtu_size = DEFAULT_MTU;
	tx_pkt_len = DEFAULT_MTU + ETH_HDR_LEN;
	rx_pkt_len = DEFAULT_MTU + ETH_HDR_LEN;
	hw_rx_pkt_len = rx_pkt_len + 8;
	
	//Config PHY
	R1000HwPhyConfig();
	
	DbgPrint("[RealtekR1000] Set MAC Reg C+CR Offset 0x82h = 0x01h\n");
	WriteMMIO8(0x82, 0x01);
	if (mcfg < MCFG_METHOD_3)
	{
		DbgPrint("[RealtekR1000] Set PCI Latency=0x40\n");
		pciDev->configWrite8(kIOPCIConfigLatencyTimer, 0x40);
	}
	
	if (mcfg == MCFG_METHOD_2)
	{
		DbgPrint("[RealtekR1000] Set MAC Reg C+CR Offset 0x82h = 0x01h\n");
		WriteMMIO8(0x82, 0x01);
		DbgPrint("[RealtekR1000] Set PHY Reg 0x0bh = 0x00h\n");
		WriteGMII32(0x0b, 0x0000);	//w 0x0b 15 0 0
	}
	
	int speed_opt = SPEED_100;
	int duplex_opt = DUPLEX_FULL;
	int autoneg_opt = AUTONEG_ENABLE;
	int val = 0;
		
	// if TBI is not endbled
	if (!(ReadMMIO8(PHYstatus) & TBI_Enable))
	{
		val = ReadGMII32(PHY_AUTO_NEGO_REG);
		val |= PHY_Cap_PAUSE | PHY_Cap_ASYM_PAUSE ;

		R1000SetMedium(speed_opt, duplex_opt, autoneg_opt);
	}// end of TBI is not enabled
	else
	{
		IODelay(100);
		DbgPrint("1000Mbps Full-duplex operation, TBI Link %s!\n", (ReadMMIO32(TBICSR) & TBILinkOK) ? "OK" : "Failed" );
	}// end of TBI is not enabled
	
#ifdef DEBUG	
	if (ReadMMIO8(PHYstatus) & LinkStatus)
	{
		DbgPrint("[RealtekR1000] Link Status: %s\n","Linked");

		if (ReadMMIO8(PHYstatus) & _1000Mbps) DbgPrint("[RealtekR1000] Link Speed: 1000Mbps\n");
		else if(ReadMMIO8(PHYstatus) & _100Mbps) DbgPrint("[RealtekR1000] Link Speed: 100Mbps\n");
		else if(ReadMMIO8(PHYstatus) & _10Mbps) DbgPrint("[RealtekR1000] Link Speed: 10Mbps\n");

		DbgPrint("[RealtekR1000] Duplex mode: %s\n", ReadMMIO8(PHYstatus) & FullDup ? "Full-Duplex" : "Half-Duplex");
	}
	else
	{
		DbgPrint("[RealtekR1000] Link Status: %s\n","Not Linked");
	}
#endif

	return true;
}
	
bool RealtekR1000::R1000StopBoard()
{
	return true;
}

bool RealtekR1000::R1000SetSpeedDuplex(ulong anar, ulong gbcr, ulong bmcr)
 {
	unsigned int i = 0;
	unsigned int bmsr;

	WriteGMII32(PHY_AUTO_NEGO_REG, anar);
	WriteGMII32(PHY_1000_CTRL_REG, gbcr);
	WriteGMII32(PHY_CTRL_REG, bmcr);

	for(i=0; i<10000; i++)
	{
		bmsr = ReadGMII32(PHY_STAT_REG);
		if (bmsr & PHY_Auto_Neco_Comp) return true;
	}
	return false;	
}


bool RealtekR1000::R1000SetMedium(ushort speed, uchar duplex, uchar autoneg)
{
	ulong anar=0, gbcr=0, bmcr=0, val=0;

	val = ReadGMII32(PHY_AUTO_NEGO_REG);
	val |= PHY_Cap_PAUSE | PHY_Cap_ASYM_PAUSE ;

	bmcr = PHY_Restart_Auto_Nego|PHY_Enable_Auto_Nego;

	if(autoneg == AUTONEG_ENABLE)
	{
		this->autoneg = AUTONEG_ENABLE;
		anar = PHY_Cap_10_Half | PHY_Cap_10_Full | PHY_Cap_100_Half | PHY_Cap_100_Full;
		gbcr = PHY_Cap_1000_Half | PHY_Cap_1000_Full;
	}
	else
	{
		this->autoneg = AUTONEG_DISABLE;
		if(speed == SPEED_1000)
		{
			this->speed = SPEED_1000;
			this->duplex = DUPLEX_FULL;

			anar = PHY_Cap_10_Half | PHY_Cap_10_Full | PHY_Cap_100_Half | PHY_Cap_100_Full;
			if ((mcfg == MCFG_METHOD_13) || (mcfg == MCFG_METHOD_14) ||
				 (mcfg == MCFG_METHOD_15)) gbcr = PHY_Cap_Null;
			else gbcr = PHY_Cap_1000_Half | PHY_Cap_1000_Full;
		}
		else if ((speed == SPEED_100) && (duplex == DUPLEX_FULL))
		{
			this->speed = SPEED_100;
			this->duplex = DUPLEX_FULL;

			anar = PHY_Cap_10_Half | PHY_Cap_10_Full | PHY_Cap_100_Half | PHY_Cap_100_Full;
			gbcr = PHY_Cap_Null;
		}
		else if (( speed == SPEED_100) && (duplex == DUPLEX_HALF))
		{
			this->speed = SPEED_100;
			this->duplex = DUPLEX_HALF;

			anar = PHY_Cap_10_Half | PHY_Cap_10_Full | PHY_Cap_100_Half;
			gbcr = PHY_Cap_Null;
		}
		else if((speed == SPEED_10) && (duplex == DUPLEX_FULL))
		{
			this->speed = SPEED_10;
			this->duplex = DUPLEX_FULL;

			anar = PHY_Cap_10_Half | PHY_Cap_10_Full;
			gbcr = PHY_Cap_Null;
		}
		else if ((speed == SPEED_10) && (duplex == DUPLEX_HALF))
		{
			this->speed = SPEED_10;
			this->duplex = DUPLEX_HALF;

			anar = PHY_Cap_10_Half;
			gbcr = PHY_Cap_Null;
		}
		else
		{
			this->speed = SPEED_100;
			this->duplex = DUPLEX_FULL;

			anar = PHY_Cap_10_Half|PHY_Cap_10_Full|PHY_Cap_100_Half|PHY_Cap_100_Full;
			gbcr = PHY_Cap_Null;
		}
	}

	//enable flow control
	anar |=  val & 0xC1F;

	return R1000SetSpeedDuplex(anar, gbcr, bmcr);
}


void RealtekR1000::R1000HwPhyReset()
{
	int val, phy_reset_expiretime = 50;

	DbgPrint("[RealtekR1000] Reset PHY\n");
	val = (ReadGMII32(0) | 0x8000) & 0xffff;
	WriteGMII32(0, val);

	do //waiting for phy reset
	{
		if (ReadGMII32(0) & 0x8000)
		{
			phy_reset_expiretime --;
			IODelay(100);
		}
		else
		{
			break;
		}
	}
	while (phy_reset_expiretime >= 0);
}

void RealtekR1000::R1000HwPhyConfig()
{
	DbgPrint("[RealtekR1000] PHY config\n");

	if (mcfg == MCFG_METHOD_4 )
	{
#if 0
		WriteGMII32(0x1F, 0x0001);
		WriteGMII32(0x1b, 0x841e);
		WriteGMII32(0x0e, 0x7bfb);
		WriteGMII32(0x09, 0x273a);
#endif
		WriteGMII32(0x1F, 0x0002);
		WriteGMII32(0x01, 0x90D0);
		WriteGMII32(0x1F, 0x0000);
	}
	else if ((mcfg == MCFG_METHOD_2) || (mcfg == MCFG_METHOD_3))
	{
		WriteGMII32(0x1f, 0x0001);
		WriteGMII32(0x06, 0x006e);
		WriteGMII32(0x08, 0x0708);
		WriteGMII32(0x15, 0x4000);
		WriteGMII32(0x18, 0x65c7);

		WriteGMII32(0x1f, 0x0001);
		WriteGMII32(0x03, 0x00a1);
		WriteGMII32(0x02, 0x0008);
		WriteGMII32(0x01, 0x0120);
		WriteGMII32(0x00, 0x1000);
		WriteGMII32(0x04, 0x0800);
		WriteGMII32(0x04, 0x0000);

		WriteGMII32(0x03, 0xff41);
		WriteGMII32(0x02, 0xdf60);
		WriteGMII32(0x01, 0x0140);
		WriteGMII32(0x00, 0x0077);
		WriteGMII32(0x04, 0x7800);
		WriteGMII32(0x04, 0x7000);

		WriteGMII32(0x03, 0x802f);
		WriteGMII32(0x02, 0x4f02);
		WriteGMII32(0x01, 0x0409);
		WriteGMII32(0x00, 0xf0f9);
		WriteGMII32(0x04, 0x9800);
		WriteGMII32(0x04, 0x9000);

		WriteGMII32(0x03, 0xdf01);
		WriteGMII32(0x02, 0xdf20);
		WriteGMII32(0x01, 0xff95);
		WriteGMII32(0x00, 0xba00);
		WriteGMII32(0x04, 0xa800);
		WriteGMII32(0x04, 0xa000);

		WriteGMII32(0x03, 0xff41);
		WriteGMII32(0x02, 0xdf20);
		WriteGMII32(0x01, 0x0140);
		WriteGMII32(0x00, 0x00bb);
		WriteGMII32(0x04, 0xb800);
		WriteGMII32(0x04, 0xb000);

		WriteGMII32(0x03, 0xdf41);
		WriteGMII32(0x02, 0xdc60);
		WriteGMII32(0x01, 0x6340);
		WriteGMII32(0x00, 0x007d);
		WriteGMII32(0x04, 0xd800);
		WriteGMII32(0x04, 0xd000);

		WriteGMII32(0x03, 0xdf01);
		WriteGMII32(0x02, 0xdf20);
		WriteGMII32(0x01, 0x100a);
		WriteGMII32(0x00, 0xa0ff);
		WriteGMII32(0x04, 0xf800);
		WriteGMII32(0x04, 0xf000);

		WriteGMII32(0x1f, 0x0000);
		WriteGMII32(0x0b, 0x0000);
		WriteGMII32(0x00, 0x9200);
#if 0
		WriteGMII32(0x1F, 0x0001);
		WriteGMII32(0x15, 0x1000);
		WriteGMII32(0x18, 0x65C7);
		WriteGMII32(0x04, 0x0000);
		WriteGMII32(0x03, 0x00A1);
		WriteGMII32(0x02, 0x0008);
		WriteGMII32(0x01, 0x1020);
		WriteGMII32(0x00, 0x1000);
		WriteGMII32(0x04, 0x0800);
		WriteGMII32(0x04, 0x0000);
		WriteGMII32(0x04, 0x7000);
		WriteGMII32(0x03, 0xFF41);
		WriteGMII32(0x02, 0xDE60);
		WriteGMII32(0x01, 0x0140);
		WriteGMII32(0x00, 0x0077);
		WriteGMII32(0x04, 0x7800);
		WriteGMII32(0x04, 0x7000);
		WriteGMII32(0x04, 0xA000);
		WriteGMII32(0x03, 0xDF01);
		WriteGMII32(0x02, 0xDF20);
		WriteGMII32(0x01, 0xFF95);
		WriteGMII32(0x00, 0xFA00);
		WriteGMII32(0x04, 0xA800);
		WriteGMII32(0x04, 0xA000);
		WriteGMII32(0x04, 0xB000);
		WriteGMII32(0x03, 0xFF41);
		WriteGMII32(0x02, 0xDE20);
		WriteGMII32(0x01, 0x0140);
		WriteGMII32(0x00, 0x00BB);
		WriteGMII32(0x04, 0xB800);
		WriteGMII32(0x04, 0xB000);
		WriteGMII32(0x04, 0xF000);
		WriteGMII32(0x03, 0xDF01);
		WriteGMII32(0x02, 0xDF20);
		WriteGMII32(0x01, 0xFF95);
		WriteGMII32(0x00, 0xBF00);
		WriteGMII32(0x04, 0xF800);
		WriteGMII32(0x04, 0xF000);
		WriteGMII32(0x04, 0x0000);
		WriteGMII32(0x1F, 0x0000);
		WriteGMII32(0x0B, 0x0000);
#endif
	}
}

void RealtekR1000::R1000HwStart()
{
	u32 i;
	u8 i8;
	u16 i16;
	
	if ((mcfg != MCFG_METHOD_5) && (mcfg != MCFG_METHOD_11) &&
	   (mcfg != MCFG_METHOD_12) && (mcfg != MCFG_METHOD_13) &&
	   (mcfg != MCFG_METHOD_14) && (mcfg != MCFG_METHOD_15))
	{
		/* Soft reset the chip. */
		WriteMMIO8 ( ChipCmd, CmdReset);

		/* Check that the chip has finished the reset. */
		for (i = 1000; i > 0; i--)
		{
			if ((ReadMMIO8( ChipCmd ) & CmdReset) == 0) break;
			else IODelay(10);
		}
			
		WriteMMIO8(Cfg9346, Cfg9346_Unlock);
		WriteMMIO8(ChipCmd, CmdTxEnb | CmdRxEnb);
		WriteMMIO8(ETThReg, ETTh);
			
		// For gigabit rtl8169
		WriteMMIO16(RxMaxSize, static_cast<unsigned short>(hw_rx_pkt_len));

		// Set Rx Config register
		i = r1000_rx_config | (ReadMMIO32(RxConfig) & rtl_chip_info[chipset].RxConfigMask);
		WriteMMIO32(RxConfig, i);
			
		/* Set DMA burst size and Interframe Gap Time */
		WriteMMIO32(TxConfig, (TX_DMA_BURST << TxDMAShift) | (InterFrameGap << TxInterFrameGapShift));

		WriteMMIO16(CPlusCmd, ReadMMIO16(CPlusCmd));
		
		if(mcfg == MCFG_METHOD_2 || mcfg == MCFG_METHOD_3)
		{
			WriteMMIO16(CPlusCmd, (ReadMMIO16(CPlusCmd) | (1 << 14) | (1 << 3)));
			DbgPrint("[RealtekR1000] Set MAC Reg C+CR Offset 0xE0: bit-3 and bit-14\n");
		}
		else
		{
			WriteMMIO16(CPlusCmd, (ReadMMIO16(CPlusCmd) | (1 << 3)));
			DbgPrint("[RealtekR1000] Set MAC Reg C+CR Offset 0xE0: bit-3.\n");
		}
	
		{
			WriteMMIO16(0xE2,0x0000);
		}
		
		//TO-DO: FIX-ME: Fixed?
		
		cur_rx = 0;

		WriteMMIO32(TxDescStartAddr, txdesc_phy_dma_addr);
		WriteMMIO32(TxDescStartAddr + 4, 0x00);
		WriteMMIO32(RxDescStartAddr, rxdesc_phy_dma_addr);
		WriteMMIO32(RxDescStartAddr + 4, 0x00);

		WriteMMIO8(Cfg9346, Cfg9346_Lock);
		IODelay(10);
		
		WriteMMIO32 ( RxMissed, 0 );

		//TO-DO: FIX-ME
		//r1000_set_rx_mode(netdev);

		WriteMMIO16(MultiIntr, ReadMMIO16(MultiIntr) & 0xF000);
		WriteMMIO16(IntrMask, r1000_intr_mask);
	}
	else
	{
		/* Soft reset the chip. */
		WriteMMIO8(ChipCmd, CmdReset);

		/* Check that the chip has finished the reset. */
		for (i = 1000; i > 0; i--)
		{
			if ((ReadMMIO8(ChipCmd) & CmdReset) == 0) break;
			else IODelay(10); //Microseconds
		}	
		
		if (mcfg == MCFG_METHOD_13 )
		{
			pciDev->configWrite16(0x68, 0x00);
			pciDev->configWrite16(0x69, 0x08);
		}
		
		if (mcfg == MCFG_METHOD_5)
		{
			i8 = ReadMMIO8(Config2);
			i8 = i8 & 0x07;
			if (i8 && 0x01)	WriteMMIO32(Off7Ch, 0x0007FFFF);
	
			i = 0x0007FF00;
			WriteMMIO32(Off7Ch, i);

			i16 = pciDev->configRead16(0x04);
			i16 = i16 & 0xEF;
			pciDev->configWrite16(0x04, i16);
		}
		
		WriteMMIO8(Cfg9346, Cfg9346_Unlock);
		WriteMMIO8(ETThReg, ETTh);
		
		// For gigabit rtl8169
		WriteMMIO16(RxMaxSize, static_cast<unsigned short>(hw_rx_pkt_len));

		WriteMMIO16(CPlusCmd, ReadMMIO16(CPlusCmd));
		
		if (mcfg == MCFG_METHOD_2 || mcfg == MCFG_METHOD_3)
		{
			WriteMMIO16(CPlusCmd, (ReadMMIO16(CPlusCmd) | (1 << 14) | (1 << 3)));
			DbgPrint("[RealtekR1000] Set MAC Reg C+CR Offset 0xE0: bit-3 and bit-14\n");
		}
		else
		{
			WriteMMIO16(CPlusCmd, (ReadMMIO16(CPlusCmd) | (1 << 3)));
			DbgPrint("[RealtekR1000] Set MAC Reg C+CR Offset 0xE0: bit-3.\n");
		}
		
		{
			WriteMMIO16(0xE2,0x0000);
		}
		
		//TO-DO: FIX-ME: Fixed?
		cur_rx = 0;

		WriteMMIO32(TxDescStartAddr, txdesc_phy_dma_addr);
		WriteMMIO32(TxDescStartAddr + 4, 0x00);
		WriteMMIO32(RxDescStartAddr, rxdesc_phy_dma_addr);
		WriteMMIO32(RxDescStartAddr + 4, 0x00);
		WriteMMIO8(ChipCmd, CmdTxEnb | CmdRxEnb);
		// Set Rx Config register
		i = r1000_rx_config | (ReadMMIO32(RxConfig) & rtl_chip_info[chipset].RxConfigMask);
		WriteMMIO32(RxConfig, i);

		/* Set DMA burst size and Interframe Gap Time */
		WriteMMIO32(TxConfig, (TX_DMA_BURST << TxDMAShift) | (InterFrameGap << TxInterFrameGapShift));

		WriteMMIO8(Cfg9346, Cfg9346_Lock);
		IODelay(10);
		
		WriteMMIO32 (RxMissed, 0);

		//TO-DO: FIX-ME
		//r1000_set_rx_mode(netdev);

		WriteMMIO16(MultiIntr, ReadMMIO16(MultiIntr) & 0xF000);

		WriteMMIO16(IntrMask, r1000_intr_mask);
	}
	
	//TO-DO: FIX-ME
	//netif_start_queue(netdev);
}

ulong RealtekR1000::ether_crc(int length, unsigned char *data)
{
	int crc = -1;

	while (--length >= 0) 
	{
		unsigned char current_octet = *data++;
		int bit;
		for (bit = 0; bit < 8; bit++, current_octet >>= 1)
			crc = (crc << 1) ^ ((crc < 0) ^ (current_octet & 1) ? ethernet_polynomial : 0);
	}

	return crc;
}


bool RealtekR1000::AllocateDescriptorsMemory()
{
	//Allocating descriptor memory
	IOByteCount len;
	
	sizeof_txdesc_space = NUM_TX_DESC * sizeof(TxDesc) + 256;
	tx_descMd = IOBufferMemoryDescriptor::withOptions(kIOMemoryPhysicallyContiguous,
														sizeof_txdesc_space,
														PAGE_SIZE);
														
	if (!tx_descMd || tx_descMd->prepare() != kIOReturnSuccess)
	{
		DbgPrint("[RealtekR1000] Couldn't allocate physical memory for tx_desc\n");
		return false;
	}
	
	txdesc_space = tx_descMd->getBytesNoCopy();
	txdesc_phy_dma_addr = tx_descMd->getPhysicalSegment(0, &len);
	
	
	sizeof_rxdesc_space = NUM_RX_DESC * sizeof(RxDesc) + 256;
	rx_descMd = IOBufferMemoryDescriptor::withOptions(kIOMemoryPhysicallyContiguous,
														sizeof_rxdesc_space,
														PAGE_SIZE);
	
	if (!rx_descMd || rx_descMd->prepare() != kIOReturnSuccess)
	{
		DbgPrint("[RealtekR1000] Couldn't allocate physical memory for rx_desc\n");
		return false;
	}
	
	rxdesc_space = rx_descMd->getBytesNoCopy();
	rxdesc_phy_dma_addr = rx_descMd->getPhysicalSegment(0, &len);

	TxDescArray = reinterpret_cast<TxDesc *>(txdesc_space);
	RxDescArray = reinterpret_cast<RxDesc *>(rxdesc_space);

	for(int i = 0; i < NUM_RX_DESC; i++)
	{
		Rx_dbuff[i] = 0;
		Rx_skbuff_Md[i] = IOBufferMemoryDescriptor::withOptions(0,
														MAX_RX_SKBDATA_SIZE,
														PAGE_SIZE);
		if (!Rx_skbuff_Md[i] || Rx_skbuff_Md[i]->prepare() != kIOReturnSuccess)
		{
			DbgPrint("[RealtekR1000] Couldn't allocate physical memory for Rx_dbuff, step %d\n", i);
			return false;
		}
		Rx_dbuff[i] = static_cast<uchar *>(Rx_skbuff_Md[i]->getBytesNoCopy());
		if (!Rx_dbuff[i])
		{
			DbgPrint("[RealtekR1000] Pointer in NULL, step %d\n", i);
			return false;
		}		
		IOByteCount len;
		Rx_skbuff_Dma[i] = Rx_skbuff_Md[i]->getPhysicalSegment(0, &len);
		RxDescArray[i].status = 0;
		RxDescArray[i].vlan_tag = 0;
		/*Rx_skbuff[i] = NULL;//allocatePacket(MAX_RX_SKBDATA_SIZE);
		Rx_skbuff_Dma[i] = static_cast<IOPhysicalAddress>(mbuf_data_to_physical(mbuf_data(Rx_skbuff[i])));*/
				
	}
	
	//Ring initialization
	for(int i = 0; i < NUM_TX_DESC; i++)
	{
		/*Tx_skbuff[i] = NULL;
		TxDescArray[i].status = 0;*/
		//Tx_skbuff_Md[i] = NULL;
		Tx_dbuff[i] = NULL;
		Tx_skbuff_Md[i] = IOBufferMemoryDescriptor::withOptions(0,
																MAX_TX_SKBDATA_SIZE,
																PAGE_SIZE);
		if (!Tx_skbuff_Md[i] || Tx_skbuff_Md[i]->prepare() != kIOReturnSuccess)
		{
			DbgPrint("[RealtekR1000] Couldn't allocate physical memory for Tx_dbuff, step %d\n", i);
			return false;
		}
		Tx_dbuff[i] = static_cast<uchar *>(Tx_skbuff_Md[i]->getBytesNoCopy());
		if (!Tx_dbuff[i])
		{
			DbgPrint("[RealtekR1000] Pointer in NULL, step %d\n", i);
			return false;
		}
		IOByteCount len;
		Tx_skbuff_Dma[i] = static_cast<IOPhysicalAddress>(Tx_skbuff_Md[i]->getPhysicalSegment(0, &len));
		TxDescArray[i].status = 0;
		TxDescArray[i].vlan_tag = 0;
		TxDescArray[i].buf_addr = OSSwapHostToLittleInt32(Tx_skbuff_Dma[i]);
		TxDescArray[i].buf_Haddr = 0;
	}
	
	for(int i = 0; i < NUM_RX_DESC; i++)
	{
		if (i == (NUM_RX_DESC - 1))
		{
			RxDescArray[i].status = OSSwapHostToLittleInt32((OWNbit | EORbit) | static_cast<unsigned long>(hw_rx_pkt_len));
		}
		else
		{
			RxDescArray[i].status = OSSwapHostToLittleInt32(OWNbit | static_cast<unsigned long>(hw_rx_pkt_len));
		}
		
		RxDescArray[i].buf_addr = OSSwapHostToLittleInt32(Rx_skbuff_Dma[i]);
		RxDescArray[i].buf_Haddr = 0;
	}

	return true;
}

void RealtekR1000::FreeDescriptorsMemory()
{
	R1000TxClear();
	if (tx_descMd)
	{
		tx_descMd->complete();
		tx_descMd->release();
		tx_descMd = NULL;
	}
	
	if (rx_descMd)
	{
		rx_descMd->complete();
		rx_descMd->release();
		rx_descMd = NULL;
	}
	
	for(int i = 0; i < NUM_RX_DESC; i++)
	{
		if (Rx_skbuff_Md[i])
		{
			Rx_skbuff_Md[i]->complete();
			Rx_skbuff_Md[i]->release();
			Rx_skbuff_Md[i] = NULL;
		}
	}
	
	for(int i = 0; i < NUM_TX_DESC; i++)
	{
		if (Tx_skbuff_Md[i])
		{
			Tx_skbuff_Md[i]->complete();
			Tx_skbuff_Md[i]->release();
			Tx_skbuff_Md[i] = NULL;
		}
	}	
}
	
bool RealtekR1000::R1000InitEventSources()
{
	DbgPrint("[RealtekR1000] RealtekR1000::R1000InitEventSources()\n");
	
	IOWorkLoop *loop = getWorkLoop(); //Needed, cause may be called before WorkLoop creation
	if (!loop) return false;
	
	transmitQueue = getOutputQueue();
	if (!transmitQueue) return false; //Sanity check
		
	intSource = IOInterruptEventSource::interruptEventSource(this, 
					OSMemberFunctionCast(IOInterruptEventSource::Action, this, &RealtekR1000::R1000Interrupt),
					pciDev);
	/*intSource = IOFilterInterruptEventSource::filterInterruptEventSource(
						this,
						OSMemberFunctionCast(IOInterruptEventSource::Action, this, &RealtekR1000::R1000Interrupt),
						OSMemberFunctionCast(IOFilterInterruptEventSource::Filter, this, &RealtekR1000::R1000FilterInterrupt),
						pciDev);*/
					
	//Adding interrupt to our workloop event sources
	if (!intSource || loop->addEventSource(intSource) != kIOReturnSuccess) return false;
	
	intSource->enable();
	
	//Registering watchdog (i.e. if timeout exceeds)
	timerSource = IOTimerEventSource::timerEventSource(this, 
					OSMemberFunctionCast(IOTimerEventSource::Action, this, &RealtekR1000::R1000TxTimeout));
					
	if (!timerSource || loop->addEventSource(timerSource) != kIOReturnSuccess) return false;
	
	return true;
}

bool RealtekR1000::R1000OpenAdapter()
{
	DbgPrint("[RealtekR1000] bool RealtekR1000::R1000OpenAdapter()\n");
	R1000HwStart();
	return true;
}

void RealtekR1000::R1000CloseAdapter()
{
	/* Stop the chip's Tx and Rx processes. */
	WriteMMIO8(ChipCmd, 0x00);

	/* Disable interrupts by clearing the interrupt mask. */
	WriteMMIO16(IntrMask, 0x0000);

	/* Update the error counts. */
	//TO-DO: FIX-ME, Realy need?
	//priv->stats.rx_missed_errors += ReadMMIO32(RxMissed);
	WriteMMIO32(RxMissed, 0);
}

void RealtekR1000::R1000TxClear()
{
	cur_tx = 0;
	dirty_tx = 0;
	for(int i = 0; i < NUM_TX_DESC; i++)
	{
		TxDescArray[i].status = 0;
		if (Tx_skbuff[i] != NULL)
		{
			freePacket(Tx_skbuff[i]);
			Tx_skbuff[i] = NULL;
		}
	}
}

//Interrupt handler
void RealtekR1000::R1000Interrupt(OSObject * client, IOInterruptEventSource * src, int count)
{
	//DbgPrint("[RealtekR1000] RealtekR1000::R1000Interrupt(OSObject * client, IOInterruptEventSource * src, int count)\n");
	
	int boguscnt = max_interrupt_work;
	unsigned int status = 0;
	unsigned int phy_status = 0;
	
	WriteMMIO16(IntrMask, 0x0000);
	
	do 
	{
		status = ReadMMIO16(IntrStatus);

		if (status == 0xFFFF) break;
		WriteMMIO16(IntrStatus, status);

		if ((status & r1000_intr_mask) == 0) break;
		R1000RxInterrupt();
		R1000TxInterrupt();
		if ((status & TxOK) && (status & TxDescUnavail)) WriteMMIO8(TxPoll, 0x40);

		phy_status = ReadMMIO8(PHYstatus);
		if (((mcfg == MCFG_METHOD_2) || (mcfg == MCFG_METHOD_3)) && (phy_status & _100Mbps))
		{
			if(status&LinkChg)
			{
				if(phy_status&LinkStatus)
				{
					WriteGMII32(0x1f, 0x0001);
					WriteGMII32(0x10, 0xf01b);
					WriteGMII32(0x1f, 0x0000);
				}
				else
				{
					WriteGMII32(0x1f, 0x0001);
					WriteGMII32(0x10, 0xf41b);
					WriteGMII32(0x1f, 0x0000);
				}
			}
		}
		
		boguscnt--;
	} 
	while (boguscnt > 0);
	
	if (boguscnt <= 0) 
	{
		//DbgPrint("[RealtekR1000] Too much work at interrupt!\n");
		WriteMMIO16(IntrStatus, 0xffff);	// Clear all interrupt sources
	}

	WriteMMIO16 ( IntrMask, r1000_intr_mask);
}

/*bool RealtekR1000::R1000FilterInterrupt(OSObject *owner, IOFilterInterruptEventSource *src)
{
	return true;
}*/

void RealtekR1000::R1000RxInterrupt()
{
	int lcur_rx;
	int pkt_size = 0 ;
	int rxdesc_cnt = 0;
	
	struct __mbuf *rx_skb = NULL;
	struct	RxDesc	*rxdesc;
	
	lcur_rx = cur_rx;
	
	rxdesc = &RxDescArray[lcur_rx];
	
	while (((OSSwapLittleToHostInt32(rxdesc->status) & OWNbit) == 0) && (rxdesc_cnt < max_interrupt_work))
	{	
	    rxdesc_cnt++;

	    if(OSSwapLittleToHostInt32(rxdesc->status) & RxRES)
		{
			DbgPrint("[RealtekR1000] RX_RING_IS_FULL stalling\n");
			//TO-DO: Add error counters
			if (OSSwapLittleToHostInt32(rxdesc->status) & (RxRWT|RxRUNT)) ;//priv->stats.rx_length_errors++;
			if (OSSwapLittleToHostInt32(rxdesc->status) & RxCRC) ;//priv->stats.rx_crc_errors++;
	    }
		else
		{
			pkt_size=static_cast<int>(OSSwapLittleToHostInt32(rxdesc->status) & 0x00001FFF) - 4;
			
			if (pkt_size > rx_pkt_len )
			{
				DbgPrint("[RealtekR1000] Error -- Rx packet size() > mtu()+14\n");
				pkt_size = rx_pkt_len;
			}
			
			//rx_skb = Rx_skbuff[lcur_rx];
			rx_skb = allocatePacket(MAX_RX_SKBDATA_SIZE);
			if (!rx_skb) continue;
			bcopy(Rx_dbuff[lcur_rx], mbuf_data(rx_skb), pkt_size);
			
			if (rx_skb != NULL)
			{
				mbuf_setlen(rx_skb, pkt_size);
				//TO-DO: Add network stack notification
				DbgPrint("[RealtekR1000] Receive: packet len %d, mised packets %d\n", pkt_size, ReadMMIO32(RxMissed));
				{
#ifdef DEBUG
					/*uchar *data = static_cast<uchar *>(mbuf_data(rx_skb));
					for(int i = 0; i < pkt_size; i++)
						DbgPrint("%02x ", data[i]);
						h 
					DbgPrint("\n");*/
#endif
				}
				netif->inputPacket(rx_skb, pkt_size, IONetworkInterface::kInputOptionQueuePacket);
				netif->flushInputQueue();
			}
			else
			{
				DbgPrint("[RealtekR1000] Allocate n_skb failed!\n");
			}
			
			// Update rx descriptor
			if (lcur_rx == (NUM_RX_DESC - 1))
			{
				RxDescArray[lcur_rx].status = OSSwapHostToLittleInt32((OWNbit | EORbit) | static_cast<unsigned long>(hw_rx_pkt_len));
			}
			else
			{
				RxDescArray[lcur_rx].status = OSSwapHostToLittleInt32(OWNbit | static_cast<unsigned long>(hw_rx_pkt_len));
			}
		}
		
	    lcur_rx = (lcur_rx +1) % NUM_RX_DESC;
	    rxdesc = &RxDescArray[lcur_rx];
	}
	
	if (rxdesc_cnt >= max_interrupt_work)
	{
		DbgPrint("[RealtekR1000]  Too much work at Rx interrupt.\n");
	}

	cur_rx = lcur_rx;
}

void RealtekR1000::R1000TxInterrupt()
{
	unsigned long ldirty_tx, tx_left=0;
	int entry = cur_tx % NUM_TX_DESC;
    	int txloop_cnt = 0;
			
	ldirty_tx = dirty_tx;
	tx_left = cur_tx - ldirty_tx;
	DbgPrint("[RealtekR1000] R1000TxInterrupt cur_tx %d, dirty_tx %d, tx_left %d\n", cur_tx, dirty_tx, tx_left);
	
	while ((tx_left > 0) && (txloop_cnt < max_interrupt_work))
	{
		if ((OSSwapLittleToHostInt32(TxDescArray[entry].status) & OWNbit) == 0)
		{
			if (Tx_skbuff[ldirty_tx % NUM_TX_DESC] != NULL)
			{
				DbgPrint("[RealtekR1000] Send: R1000TxInterrupt, packet sended to the network, packet size %d, phys addr %x\n", mbuf_len(Tx_skbuff[ldirty_tx % NUM_TX_DESC]), OSSwapLittleToHostInt32(RxDescArray[ldirty_tx % NUM_TX_DESC].buf_addr));
				{
#ifdef DEBUG
					/*uchar *data = Tx_dbuff[ldirty_tx % NUM_TX_DESC];
					for(int i = 0; i < mbuf_len(Tx_skbuff[ldirty_tx % NUM_TX_DESC]); i++)
						DbgPrint("%02x ", data[i]);
					
					DbgPrint("\n");*/
#endif
				}
				freePacket(Tx_skbuff[ldirty_tx % NUM_TX_DESC]);
				Tx_skbuff[ldirty_tx % NUM_TX_DESC] = NULL;
			}
			else
			{
				DbgPrint("[RealtekR1000] Tx_skbuff[ldirty_tx % NUM_TX_DESC] is NULL!\n");
			}
			/*Tx_skbuff_Md[ldirty_tx % NUM_TX_DESC]->complete();
			Tx_skbuff_Md[ldirty_tx % NUM_TX_DESC]->release();
			Tx_skbuff_Md[ldirty_tx % NUM_TX_DESC] = NULL;*/

			ldirty_tx++;
			tx_left--;
			entry++;
		}
		txloop_cnt++;
	}
	
	if (dirty_tx != ldirty_tx) 
	{
		dirty_tx = ldirty_tx;
		transmitQueue->start();
	}
}

void RealtekR1000::R1000TxTimeout(OSObject *owner, IOTimerEventSource * timer)
{
	DbgPrint("[RealtekR1000] RealtekR1000::R1000TxTimeout(OSObject *owner, IOTimerEventSource * timer)\n");
	
	uchar tmp8;

	/* disable Tx, if not already */
	tmp8 = ReadMMIO8( ChipCmd );
	if (tmp8 & CmdTxEnb)
	{
		WriteMMIO8 (ChipCmd, tmp8 & ~CmdTxEnb);
	}
	
	/* Disable interrupts by clearing the interrupt mask. */
	WriteMMIO16(IntrMask, 0x0000);
	
	R1000TxClear();
	R1000HwStart();
	transmitQueue->start();
}




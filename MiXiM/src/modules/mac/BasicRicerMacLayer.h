/*
 * BasicRicerMacLayer.h
 *
 *  Created on: Jan 7, 2016
 *      Author: KISIDO
 */

#ifndef BASICRICERMACLAYER_H_
#define BASICRICERMACLAYER_H_




#include <string>
#include <sstream>
#include <vector>
#include <list>

#include "MiXiMDefs.h"
#include "BaseMacLayer.h"
#include <DroppedPacket.h>
#include "MobilityBase.h"
#include "BaseConnectionManager.h"
#include "NicEntry.h"


class MacPkt;

/**
 * @brief Implementation of B-MAC (called also Berkeley MAC, Low Power
 * Listening or LPL).
 *
 * The protocol works as follows: each node is allowed to sleep for
 * slotDuration. After waking up, it first checks the channel for ongoing
 * transmissions.
 * If a transmission is catched (a preamble is received), the node stays awake
 * for at most slotDuration and waits for the actual data packet.
 * If a node wants to send a packet, it first sends preambles for at least
 * slotDuration, thus waking up all nodes in its transmission radius and
 * then sends out the data packet. If a mac-level ack is required, then the
 * receiver sends the ack immediately after receiving the packet (no preambles)
 * and the sender waits for some time more before going back to sleep.
 *
 * B-MAC is designed for low traffic, low power communication in WSN and is one
 * of the most widely used protocols (e.g. it is part of TinyOS).
 * The finite state machine of the protocol is given in the below figure:
 *
 * \image html BMACFSM.png "B-MAC Layer - finite state machine"
 *
 * A paper describing this implementation can be found at:
 * http://www.omnet-workshop.org/2011/uploads/slides/OMNeT_WS2011_S5_C1_Foerster.pdf
 *
 * @class BMacLayer
 * @ingroup macLayer
 * @author Anna Foerster
 *
 */
class MIXIM_API BasicRicerMacLayer : public BaseMacLayer
{
  private:
    /** @brief Copy constructor is not allowed.
     */
    BasicRicerMacLayer(const BasicRicerMacLayer&);
    /** @brief Assignment operator is not allowed.
     */
    BasicRicerMacLayer& operator=(const BasicRicerMacLayer&);

  public:
    BasicRicerMacLayer()
        : BaseMacLayer()
        , macQueue()
        , nbTxDataPackets(0), nbTxPreambles(0), nbRxDataPackets(0), nbRxPreambles(0)
        , nbMissedAcks(0), nbRecvdAcks(0), nbDroppedDataPackets(0), nbTxAcks(0)
        , macState(INIT)
        , lastDataPktSrcAddr()
        , lastDataPktDestAddr()
        , txAttempts(0)
        , droppedPacket()
        , queueLength(0)
        , animation(false)
        , slotDuration(0), bitrate(0), checkInterval(0), txPower(0)
        , useMacAcks(0)
        , maxTxAttempts(0)
    {}
    virtual ~BasicRicerMacLayer();

    /** @brief Initialization of the module and some variables*/
    virtual void initialize(int);

    /** @brief Delete all dynamically allocated objects of the module*/
    virtual void finish();

    /** @brief Handle messages from lower layer */
    virtual void handleLowerMsg(cMessage*);

    /** @brief Handle messages from upper layer */
    virtual void handleUpperMsg(cMessage*);

    /** @brief Handle self messages such as timers */
    virtual void handleSelfMsg(cMessage*);

    /** @brief Handle control messages from lower layer */
    virtual void handleLowerControl(cMessage *msg);

  protected:
    typedef std::list<macpkt_ptr_t> MacQueue;

    /** @brief A queue to store packets from upper layer in case another
    packet is still waiting for transmission.*/
    MacQueue macQueue;

    /** @name Different tracked statistics.*/
    /*@{*/
    long nbTxDataPackets;
    long nbTxPreambles;
    long nbTxSuccPackets;
    long nbTxForwardPackets;

    int forwarded;

    long nbRxDataPackets;
    long nbRxColPackets;


    long nbRxPreambles;
    long nbMissedAcks;
    long nbRecvdAcks;
    long nbDroppedDataPackets;
    long nbTxAcks;


    double arrLatencyNode[10];
    long arrTotalPacketNode[10];

    int nbWakeUps;

    double idleListenning;
    double startIdle, endIdle, durationIdle, totalIdle;
    //double totalEnergy,
    double slotEnergy;

    double consumedEnergy;

    double energyCBT, energyWUB, energyDR, energyDT, energyACK, energyCOL;
    double rxPower;

    FILE* recordFile;
    FILE* recordMatlabFile;

    double beaconTimeout, dataTimeout, ackTimeout;
    double ccaBeacon, ccaData;
    double maxIdleBeacon;


   cOutVector energyState;
   cOutVector slotVector;

    bool isConnected = false;
    LAddress::L2Type parentAddress = MACAddress("ff:ff:ff:ff:ff:ff");
    int parentHops = - 1;
    MACAddress destMacAdd;


    enum States {
        INIT,   //0
        Rx_FINISH_BEACON,
        Rx_WAIT_DATA,
        Rx_FINISH_ACK,
        Rx_SLEEP,

        Tx_WAIT_BEACON,
        Tx_CCA,
        Tx_FINISH_DATA,
        Tx_WAIT_ACK,
        Tx_SLEEP,

        RELAY_FINISH_BEACON,
        RELAY_WAIT_DATA,
        RELAY_FINISH_ACK
      };
    /** @brief The current state of the protocol */
    States macState, preState;

    /** @brief Types of messages (self messages and packets) the node can
     * process **/
    enum TYPES {
        RICER_BEACON, //0
        RICER_DATA, //1
        RICER_ACK, //2

        TX_FINISH, //3
        DATA_COLLISION, //4
        RICER_TIME_OUT, //5
        RICER_START,
        RICER_WAKEUP
    };

    cMessage* data_collision;
    cMessage* tx_finish;

    // messages used in the FSM
    cMessage* ricer_start;
    cMessage* ricer_time_out;
    cMessage* ricer_wakeup;


    /** @name Help variables for the acknowledgment process. */
    /*@{*/
    LAddress::L2Type lastDataPktSrcAddr;
    LAddress::L2Type lastDataPktDestAddr;
    int              txAttempts;

    const char *hexAdd = "ff:ff:ff:ff:ff:f0";


    /** @brief Inspect reasons for dropped packets */
    DroppedPacket droppedPacket;

    int nodeID;
    double deployTime;
    int Twi = 5;
    /** @brief The maximum length of the queue */
    unsigned int queueLength;
    /** @brief Animate (colorize) the nodes.
     *
     * The color of the node reflects its basic status (not the exact state!)
     * BLACK - node is sleeping
     * GREEN - node is receiving
     * YELLOW - node is sending
     */

    bool animation;
    /** @brief The duration of the slot in secs. */
    double slotDuration;

    double i;
    /** @brief The bitrate of transmission */
    double bitrate;
    /** @brief The duration of CCA */
    double checkInterval;
    /** @brief Transmission power of the node */
    double txPower;
    /** @brief Use MAC level acks or not */
    bool useMacAcks;
    /** @brief Maximum transmission attempts per data packet, when ACKs are
     * used */
    int maxTxAttempts;
    /** @brief Gather stats at the end of the simulation */
    bool stats;

    /** @brief Possible colors of the node for animation */
    enum BMAC_COLORS {
        GREEN = 1,
        BLUE = 2,
        RED = 3,
        BLACK = 4,
        YELLOW = 5
    };

    /** @brief Internal function to change the color of the node */
    void changeDisplayColor(BMAC_COLORS color);

    /** @brief Internal function to send the first packet in the queue */
    void sendDataPacket();

    /** @brief Internal function to send an ACK */
    void sendAckPacket();

    /** @brief Internal function to send a Beacon */
    void sendBeaconPacket();

    /** @brief Internal function to forward a packet */
    void forwardPacket(macpkt_ptr_t packet);


    void calConsumedEnergy(cMessage* msg);

    /** @brief Internal function to attach a signal to the packet */
    void attachSignal(macpkt_ptr_t macPkt);

    /** @brief Internal function to add a new packet from upper to the queue */
    bool addToQueue(cMessage * msg);

    void generatePacket();

    /** @brief Handle self messages such as timers */
    void handleSelfMsgTODO(cMessage*);

};




#endif /* BASICRICERMACLAYER_H_ */

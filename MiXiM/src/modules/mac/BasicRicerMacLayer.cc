/*
 * BasicRicerMacLayer.cc
 *
 *  Created on: Jan 7, 2016
 *      Author: KISIDO
 */


#include "BasicRicerMacLayer.h"

#include <cassert>

//#include <random>
//#include <iostream>

#include <iostream>
#include <fstream>

#include "FWMath.h"
#include "MacToPhyControlInfo.h"
#include "BaseArp.h"
#include "BaseConnectionManager.h"
#include "PhyUtils.h"
#include "MacPkt_m.h"
#include "MacToPhyInterface.h"

Define_Module( BasicRicerMacLayer )

int countvect = 0;
long totalEnergy;

 double residualCapacity = 1000;

void BasicRicerMacLayer::initialize(int stage)
{
    BaseMacLayer::initialize(stage);
    if (stage == 0) {
        BaseLayer::catDroppedPacketSignal.initialize();

        nodeID = static_cast<int>(findHost()->getAncestorPar("nodeID"));

        //objPowerManager = static_cast<WindPowerManager*>(findHost()->getSubmodule("powermanager"));

        //objPowerManager = static_cast<BasePowerManager*>(findHost()->getSubmodule("powermanager"));






        queueLength     = hasPar("queueLength")     ? par("queueLength")   : 10;
        animation       = hasPar("animation")       ? par("animation")     : true;
        slotDuration    = hasPar("slotDuration")    ? par("slotDuration")  : 1.;
        bitrate         = hasPar("bitrate")         ? par("bitrate")       : 15360.;
        headerLength    = hasPar("headerLength")    ? par("headerLength")  : 10.;
        checkInterval   = hasPar("checkInterval")   ? par("checkInterval") : 0.1;
        txPower         = hasPar("txPower")         ? par("txPower")       : 50.;
        useMacAcks      = hasPar("useMACAcks")      ? par("useMACAcks")    : false;
        maxTxAttempts   = hasPar("maxTxAttempts")   ? par("maxTxAttempts") : 2;


        rxPower         = hasPar("rxPower")         ? par("rxPower")        : 76.89;
        energyCBT       = hasPar("energyCBT")       ? par("energyCBT")      : 9.7;
        energyWUB       = hasPar("energyWUB")       ? par("energyWUB")      : 51;
        energyDT        = hasPar("energyDT")        ? par("energyDT")       : 80;
        energyDR        = hasPar("energyDR")        ? par("energyDR")       : 100;
        energyACK       = hasPar("energyACK")       ? par("energyACK")      : 18;
        energyCOL       = hasPar("energyCOL")       ? par("energyCOL")      : 118;


        beaconTimeout   = hasPar("beaconTimeout")   ? par("beaconTimeout")  : 0.2;
        dataTimeout     = hasPar("dataTimeout")     ? par("dataTimeout")    : 0.005;
        ackTimeout      = hasPar("ackTimeout")      ? par("ackTimeout")     : 0.001;

        ccaBeacon       = hasPar("ccaBeacon")       ? par("ccaBeacon")       : 0.001;
        ccaData         = hasPar("ccaData")         ? par("ccaData")        : 0.003;




        //debugEV << "headerLength: " << headerLength << ", bitrate: " << bitrate << endl;

        //stats = par("stats");

        nbDroppedDataPackets = 0;
        nbTxDataPackets = 0;
        nbTxSuccPackets = 0;
        nbTxForwardPackets = 0;


        nbMissedAcks = 0;

        nbRxDataPackets = 0;
        nbRxColPackets = 0;

        totalEnergy = 0;
        totalIdle = 0;
        slotEnergy = 0;

        nbWakeUps = 0;

        txAttempts = 0;
        lastDataPktDestAddr = LAddress::L2BROADCAST;
        lastDataPktSrcAddr  = LAddress::L2BROADCAST;

        forwarded = 0;

        macState = INIT;

        destMacAdd = MACAddress("ff:ff:ff:ff:ff:f0");


       std::string outputFileName = "end_device_";
       char convert[10];
       itoa(nodeID,convert,10);
      outputFileName.append(convert);
        outputFileName.append(".txt");
       recordFile = fopen(outputFileName.c_str(), "w");

        // init the dropped packet info
        droppedPacket.setReason(DroppedPacket::NONE);




        WATCH(macState);
    }
    else if(stage == 1) {

        ricer_start = new cMessage("ricer_start");
        ricer_start->setKind(RICER_START);

        ricer_time_out = new cMessage("ricer_time_out");
        ricer_time_out->setKind(RICER_TIME_OUT);

        tx_finish = new cMessage("tx_finish");
        tx_finish->setKind(TX_FINISH);

        data_collision = new cMessage("data_collision");
        data_collision->setKind(DATA_COLLISION);

        ricer_wakeup = new cMessage("ricer_wakeup");
        ricer_wakeup->setKind(RICER_WAKEUP);

        deployTime = static_cast<double>(findHost()->getAncestorPar("deployTime"));

        fprintf(recordFile, "Tracking of End Device ID: %d, deployed at: \n", nodeID, deployTime);

        scheduleAt(simTime() + deployTime, ricer_start);
    }
}

BasicRicerMacLayer::~BasicRicerMacLayer()
{


    MacQueue::iterator it;
    for(it = macQueue.begin(); it != macQueue.end(); ++it)
    {
        delete (*it);
    }
    macQueue.clear();
}

void BasicRicerMacLayer::finish()
{
    BaseMacLayer::finish();


    cancelAndDelete(ricer_start);
    cancelAndDelete(ricer_wakeup);
    cancelAndDelete(tx_finish);
    cancelAndDelete(data_collision);
    cancelAndDelete(ricer_time_out);
}

/**
 * Check whether the queue is not full: if yes, print a warning and drop the
 * packet. Then initiate sending of the packet, if the node is sleeping. Do
 * nothing, if node is working.
 */
void BasicRicerMacLayer::handleUpperMsg(cMessage *msg)
{

}

void BasicRicerMacLayer::handleSelfMsgTODO(cMessage *msg)
{
    switch(macState)
    {
        case INIT:
            if(msg->getKind() == RICER_START)
            {
                if(nodeID == 0)
                {
                    debugEV << "****Rx: I am the Base Station" << endl;
                    phy->setRadioState(MiximRadio::TX);
                    changeDisplayColor(GREEN);
                    sendBeaconPacket();
                    macState = Rx_FINISH_BEACON;
                    return;
                }
                else
                {
                    debugEV << "****Tx: I am the End Device" << endl;
                    phy->setRadioState(MiximRadio::RX);
                    changeDisplayColor(RED);
                    macState = Tx_WAIT_BEACON;
                    return;
                }

            }
            else
            {
                return;
            }
            break;
        case Tx_WAIT_BEACON:
            if(msg->getKind() == RICER_BEACON)
            {
                phy->setRadioState(MiximRadio::TX);
                changeDisplayColor(GREEN);
                sendDataPacket();
                macState = Tx_FINISH_DATA;
                return;
            }
            else
            {
                return;
            }
            break;

        case Tx_FINISH_DATA:
            if(msg->getKind() == TX_FINISH)
            {
                phy->setRadioState(MiximRadio::RX);
                changeDisplayColor(RED);
                macState = Tx_WAIT_BEACON;
                return;
            }
            else
            {
                return;
            }
            break;

        case Rx_FINISH_BEACON:
            if(msg->getKind() == TX_FINISH)
            {
                debugEV << "****Rx: Beacon is already sent! Waiting for a data" << endl;
                phy->setRadioState(MiximRadio::RX);
                changeDisplayColor(RED);
                macState = Rx_WAIT_DATA;
                cancelEvent(ricer_time_out);
                scheduleAt(simTime() + dataTimeout, ricer_time_out);
                return;
            }
            else
            {
                return;
            }
            break;
        case Rx_WAIT_DATA:
            if(msg->getKind() == RICER_TIME_OUT)
            {
                debugEV << "****Rx: There is no data response!!Go to sleep for 5s" << endl;
                phy->setRadioState(MiximRadio::SLEEP);
                changeDisplayColor(BLACK);
                macState = Rx_SLEEP;
                scheduleAt(simTime() + 5 + slotDuration, ricer_wakeup);
                return;
            }
            else
            {
                cancelEvent(ricer_time_out);
                debugEV << "***Rx: There is a data from address" << (macpkt_ptr_t(msg))->getSrcAddr() << " and ID " << (macpkt_ptr_t(msg))->getSrcProcId() << endl;

                macState = Rx_SLEEP;
                scheduleAt(simTime() + 5 +slotDuration, ricer_wakeup);
                return;
            }
            break;
        case Rx_SLEEP:
            if(msg->getKind() == RICER_WAKEUP)
            {
                debugEV << "****Rx: Node is woken up" << endl;
                phy->setRadioState(MiximRadio::TX);
                changeDisplayColor(GREEN);
                sendBeaconPacket();
                macState = Rx_FINISH_BEACON;
                return;
            }

            break;
    }
    opp_error("Undefined event of type %d in state %d (Radio state %d)!",
                msg->getKind(), macState, phy->getRadioState());
}

void BasicRicerMacLayer::handleLowerMsg(cMessage *msg)
{
    // simply pass the massage as self message, to be processed by the FSM.

    handleSelfMsg(msg);
}

void BasicRicerMacLayer::handleSelfMsg(cMessage *msg) //TODO
{
    if(nodeID != 0 && simTime().dbl() > deployTime)
        calConsumedEnergy(msg);

    switch (macState)
    {
        case INIT:
            if(msg->getKind() == RICER_START)
            {
                if(nodeID == 0)
                {

                    debugEV << "***********************" << endl;
                    debugEV << "**  RICER Simulation 2 **" << endl;
                    debugEV << "***********************" << endl;

                    parentHops = 0;
                    debugEV << "****Rx: I am the base station" << endl;
                    debugEV << "****Rx: check interval: " << ccaData << endl;


                    Twi = 1;
                    scheduleAt(simTime() + Twi + slotDuration, ricer_wakeup);
                    changeDisplayColor(GREEN);
                    phy->setRadioState(MiximRadio::TX);
                    sendBeaconPacket();
                    macState = Rx_FINISH_BEACON;
                }
                else
                {



                    debugEV << "****Tx: I am the end device" << endl;
                    debugEV << "****Tx: Check interval is" << ccaData << endl;

                    maxIdleBeacon = 0.2; //200ms

                    Twi = 2; //objPowerManager->intInitWakeUp;
                    cancelEvent(ricer_time_out);
                    scheduleAt(simTime() + maxIdleBeacon, ricer_time_out);

                    scheduleAt(simTime() + Twi + slotDuration, ricer_wakeup);

                    phy->setRadioState(MiximRadio::RX);
                    changeDisplayColor(RED);
                    macState = Tx_WAIT_BEACON;

                    //create a packet and add to macQueue
                    generatePacket();
                }
                return;
            }
            else
            {
                return;
            }
            break;


        case Tx_WAIT_BEACON:
            if(msg->getKind() == RICER_TIME_OUT)
            {
                debugEV << "****Tx: Waiting beacon is expired..." << endl;
                phy->setRadioState(MiximRadio::SLEEP);
                changeDisplayColor(BLACK);
                macState = Tx_SLEEP;
                maxIdleBeacon += 0.1;
                if(maxIdleBeacon >= 1)
                {
                    maxIdleBeacon = 1;
                }

                cancelEvent(ricer_wakeup);
                scheduleAt(simTime() + Twi - ccaData + slotDuration, ricer_wakeup);

                return;
            }
            else if(msg->getKind() == RICER_BEACON)
            {
                cancelEvent(ricer_time_out);
                cancelEvent(ricer_wakeup);

                if(isConnected == false) // the node select its parent
                {
                    isConnected = true;
                    parentAddress = ((macpkt_ptr_t)msg)->getSrcAddr();
                    parentHops = ((macpkt_ptr_t)msg)->getSequenceId() + 1;

                    fprintf(recordFile, "Parent node : %d\n", ((macpkt_ptr_t)msg)->getSrcProcId());
                }


                if(((macpkt_ptr_t)msg)->getSrcAddr() == parentAddress) // this is a beacon from parent
                {

                    maxIdleBeacon = 0.1;

                    //if(preState == RELAY_WAIT_DATA)
                    //{
                        scheduleAt(simTime() + Twi - ccaData - dataTimeout - ccaData + slotDuration, ricer_wakeup);
                    //}
                    //else
                    //{
                    //    scheduleAt(simTime() + Twi - checkInterval, ricer_wakeup);
                    //}
                    debugEV << "****Tx: Receive a beacon, performing CCA..." << endl;
                    cancelEvent(ricer_time_out);
                    scheduleAt(simTime() + ccaData * dblrand(), ricer_time_out);
                    phy->setRadioState(MiximRadio::RX);
                    changeDisplayColor(RED);
                    macState = Tx_CCA;
                }
                else
                {
                    debugEV << "****Tx: Receive a beacon, but not from its parent" << endl;
                    debugEV << "****Tx: Update parent can be performed here!!!!" << endl;
                }



                return;
            }
            else
            {
                return;
            }

            break;

        case Tx_CCA:
            if(msg->getKind() == RICER_TIME_OUT)
            {

                debugEV << "***Tx: CCA is completed!!!" << endl;

                changeDisplayColor(GREEN);
                phy->setRadioState(MiximRadio::TX);

                if(macQueue.size() == 0)
                {
                    debugEV << "****Tx: I am sending a packet..." << endl;
                    forwarded = 0;
                    sendDataPacket();
                }
                else
                {
                    debugEV << "****Tx: I am forwarding a packet..." << endl;
                    forwarded = 1 ;
                    forwardPacket(macQueue.front());
                }

                macState = Tx_FINISH_DATA;



                return;
            }
            else if(msg->getKind() == RICER_BEACON || msg->getKind() == RICER_DATA || msg->getKind() == RICER_ACK || msg->getKind() == DATA_COLLISION)
            {

                cancelEvent(ricer_time_out);

                debugEV << "****Tx: Channel is busy! Go to sleep" << endl;
                changeDisplayColor(BLACK);
                phy->setRadioState(MiximRadio::SLEEP);
                macState = Tx_SLEEP;

                return;
            }
            else
            {
                break;
            }
            break;
        case Tx_FINISH_DATA:
            if(msg->getKind() == TX_FINISH)
            {


                debugEV <<"****Tx: Data packet is sent" << endl;
                changeDisplayColor(RED);
                phy->setRadioState(MiximRadio::RX);
                macState = Tx_WAIT_ACK;
                cancelEvent(ricer_time_out);
                scheduleAt(simTime() + ackTimeout, ricer_time_out);



                return;
            }
            break;
        case Tx_WAIT_ACK:
            if(msg->getKind() == RICER_TIME_OUT)
            {

                debugEV << "****Tx: ACK is expired!!!" << endl;

                phy->setRadioState(MiximRadio::SLEEP);
                changeDisplayColor(BLACK);
                macState = Tx_SLEEP;

                nbMissedAcks++;

                return;
            }
            else if(msg->getKind() == RICER_ACK)
            {

                if(macQueue.size() != 0)
                {
                    macpkt_ptr_t packet_serve = macQueue.front();
                    if(packet_serve->getSrcProcId() == nodeID)
                    {
                        nbTxSuccPackets++;
                    }
                    else
                    {
                        nbTxForwardPackets++;
                    }

                    macQueue.pop_front();
                }

                cancelEvent(ricer_time_out);
                debugEV << "****Tx: Received an ACK!!!" << endl;


                macpkt_ptr_t ack = static_cast<macpkt_ptr_t>(msg);

                /*
                debugEV <<"****Rx: Average wind power is: " << ack->getSequenceId()/10.0 << endl;

                if(nbWakeUps == objPowerManager->intWakeUpsPM)
                {
                    debugEV << "****Rx: The power manager is activated..." << endl;
                    debugEV << "****Rx: Consumed energy is : " << slotEnergy << endl;

                    objWeatherInfo.windPower = ack->getSequenceId()/10.0;

                    Twi = 2; //objPowerManager->getNextWakeUp(slotEnergy, objWeatherInfo);
                    nbWakeUps = 0;
                    slotEnergy = 0;

                }

                */

                phy->setRadioState(MiximRadio::SLEEP);
                changeDisplayColor(BLACK);
                macState = Tx_SLEEP;


                delete(ack);
                return;
            }
            else
            {
                return;
            }

            break;
        case Tx_SLEEP:
            if(msg->getKind() == RICER_WAKEUP)
            {

                if(isConnected == false)
                {
                    debugEV << "****Tx: Waiting a beacon..." << endl;
                    cancelEvent(ricer_time_out);
                    scheduleAt(simTime() + maxIdleBeacon, ricer_time_out);
                    phy->setRadioState(MiximRadio::RX);
                    changeDisplayColor(RED);
                    macState = Tx_WAIT_BEACON;
                    scheduleAt(simTime() + Twi + slotDuration, ricer_wakeup);
                    nbWakeUps++;
                    return;
                }
                else
                {



                    debugEV << "****Relay: Sending a potential beacon..." << endl;
                    phy->setRadioState(MiximRadio::TX);
                    changeDisplayColor(GREEN);
                    sendBeaconPacket();
                    macState = RELAY_FINISH_BEACON;
                    scheduleAt(simTime() + Twi + slotDuration, ricer_wakeup);
                    nbWakeUps++;
                    return;
                }

            }
            else
            {
                return;
            }
            break;


        case RELAY_FINISH_BEACON:
            if(msg->getKind() == TX_FINISH)
            {
                debugEV << "****Relay: Finish a beacon, wait for a data..." << endl;
                phy->setRadioState(MiximRadio::RX);
                changeDisplayColor(RED);
                cancelEvent(ricer_time_out);
                scheduleAt(simTime() + dataTimeout, ricer_time_out);
                macState = RELAY_WAIT_DATA;
                return;
            }
            break;

        case RELAY_WAIT_DATA:
            if(msg->getKind() == RICER_TIME_OUT)
            {
                debugEV << "****Relay: There is no data" << endl;
                phy->setRadioState(MiximRadio::RX);
                changeDisplayColor(RED);
                cancelEvent(ricer_time_out);
                scheduleAt(simTime() + maxIdleBeacon, ricer_time_out);
                macState = Tx_WAIT_BEACON;

                if(macQueue.size() == 0)
                {
                    //create a packet and add to macQueue
                    generatePacket();
                }
                return;
            }
            else if(msg->getKind() == RICER_DATA)
            {
                debugEV << "****Relay: there is a data" << endl;
                cancelEvent(ricer_time_out);
                addToQueue(msg);
                phy->setRadioState(MiximRadio::TX);
                changeDisplayColor(GREEN);
                sendAckPacket();
                macState = RELAY_FINISH_ACK;
                return;
            }
            else if(msg->getKind() == DATA_COLLISION)
            {
                nbRxColPackets++;

                debugEV << "****Relay: There is data collision" << endl;
                phy->setRadioState(MiximRadio::RX);
                changeDisplayColor(RED);
                cancelEvent(ricer_time_out);
                scheduleAt(simTime() + maxIdleBeacon, ricer_time_out);
                macState = Tx_WAIT_BEACON;

                return;
            }
            else
            {
                return;
            }
            break;
        case RELAY_FINISH_ACK:
            if(msg->getKind() == TX_FINISH)
            {
                phy->setRadioState(MiximRadio::RX);
                changeDisplayColor(RED);
                cancelEvent(ricer_time_out);
                scheduleAt(simTime() + maxIdleBeacon, ricer_time_out);
                macState = Tx_WAIT_BEACON;
                return;
            }
            break;

        case Rx_FINISH_BEACON:
            if(msg->getKind() == TX_FINISH)
            {
                debugEV << "****Rx: Beacon is sent, waiting for a data" << endl;
                phy->setRadioState(MiximRadio::RX);
                changeDisplayColor(RED);
                cancelEvent(ricer_time_out);
                scheduleAt(simTime() + dataTimeout, ricer_time_out);
                macState = Rx_WAIT_DATA;
                return;
            }
            break;
        case Rx_WAIT_DATA:

            if(msg->getKind() == RICER_TIME_OUT)
            {
                debugEV << "****Rx: There is no data response" << endl;
                phy->setRadioState(MiximRadio::SLEEP);
                changeDisplayColor(BLACK);
                macState = Rx_SLEEP;
                return;
            }
            else if(msg->getKind() == RICER_DATA)
            {
                cancelEvent(ricer_time_out);

                debugEV << "****Rx: I have a data packet" << endl;

                debugEV << "****Rx: From node address: " << ((macpkt_ptr_t)msg)->getSrcAddr() << endl;
                debugEV << "****Rx: Creation time: " << ((macpkt_ptr_t)msg)->getSequenceId()<< endl;
                debugEV << "****Rx: Delay of the packet is: " << simTime().dbl()*1000 - ((macpkt_ptr_t)msg)->getSequenceId() << endl;

                fprintf(recordFile, "Packet from: %d, delay: %f\n", ((macpkt_ptr_t)msg)->getSrcProcId(), simTime().dbl()*1000 - ((macpkt_ptr_t)msg)->getSequenceId());


                //calculate the latency
                int index = ((macpkt_ptr_t)msg)->getSrcProcId();
                arrLatencyNode[index] += simTime().dbl()*1000 - ((macpkt_ptr_t)msg)->getSequenceId();
                arrTotalPacketNode[index]++;

                macState = Rx_FINISH_ACK;
                phy->setRadioState(MiximRadio::TX);
                changeDisplayColor(GREEN);
                sendAckPacket();

                nbRxDataPackets++;

                return;
            }
            else if(msg->getKind() == DATA_COLLISION)
            {
                debugEV << "****Rx: There is a data collision" << endl;


                nbRxColPackets++;
                return;
            }
            else
            {
                return;
            }
            break;
        case Rx_FINISH_ACK:
            if(msg->getKind() == TX_FINISH)
            {
                debugEV << "****Rx: ACK is already sent!!!" << endl;

                macState = Rx_SLEEP;
                phy->setRadioState(MiximRadio::SLEEP);
                changeDisplayColor(BLACK);


                return;
            }
            break;
        case Rx_SLEEP:
            if(msg->getKind() == RICER_WAKEUP)
            {
                scheduleAt(simTime() + Twi + slotDuration, ricer_wakeup);
                changeDisplayColor(GREEN);
                phy->setRadioState(MiximRadio::TX);
                sendBeaconPacket();
                macState = Rx_FINISH_BEACON;
                return;
            }
            else
            {
                return;
            }
            break;
    }
    preState = macState;
    opp_error("Undefined event of type %d in state %d (Radio state %d)!",
            msg->getKind(), macState, phy->getRadioState());
}


void BasicRicerMacLayer::handleLowerControl(cMessage *msg)
{
    // Transmission of one packet is over
    if(msg->getKind() == MacToPhyInterface::TX_OVER) {
        scheduleAt(simTime(), tx_finish);
    }
    else if (msg->getKind() == 100)
    {
        scheduleAt(simTime(), data_collision);
    }
    else {
        debugEV << "control message with wrong kind -- deleting\n";
    }
    delete msg;
}


bool BasicRicerMacLayer::addToQueue(cMessage *msg)
{
    if (macQueue.size() >= queueLength) {
        // queue is full, message has to be deleted
        /*
        debugEV << "New packet arrived, but queue is FULL, so new packet is"
                  " deleted\n";
        msg->setName("MAC ERROR");
        msg->setKind(PACKET_DROPPED);
        sendControlUp(msg);
        droppedPacket.setReason(DroppedPacket::QUEUE);
        emit(BaseLayer::catDroppedPacketSignal, &droppedPacket);
        */
        nbDroppedDataPackets++;
        macQueue.pop_front();
        //return false;
    }

    macpkt_ptr_t macPkt = new MacPkt(msg->getName());
    //cObject *const cInfo = msg->removeControlInfo();

    //delete cInfo;

    macPkt->setBitLength(((macpkt_ptr_t)msg)->getBitLength());
    macPkt->setSrcProcId(((macpkt_ptr_t)msg)->getSrcProcId());
    macPkt->setSequenceId(((macpkt_ptr_t)msg)->getSequenceId());


    macPkt->setSrcAddr(((macpkt_ptr_t)msg)->getSrcAddr());
    macPkt->setDestAddr(((macpkt_ptr_t)msg)->getDestAddr());

    macPkt->setKind(((macpkt_ptr_t)msg)->getKind());




    assert(static_cast<cPacket*>(msg));

    //macPkt->encapsulate(static_cast<cPacket*>(msg));

    macQueue.push_back(macPkt);



    debugEV << "Max queue length: " << queueLength << ", packet put in queue"
              "\n  queue size: " << macQueue.size() << " macState: "
              << macState
              << " source ID : " <<  ((macpkt_ptr_t)macPkt)->getSrcProcId()
              << " sequence ID : " << ((macpkt_ptr_t)msg)->getSequenceId()
              <<  endl;


    /*
    macpkt_ptr_t  abc = macQueue.front();

    debugEV << "Source ID: " << abc->getSrcProcId() << " Sequence ID: " << abc->getSequenceId() << endl;
    debugEV << "MAC Address: " << abc->getDestAddr() << endl;

    double creation_time = abc->getCreationTime().dbl();
    debugEV << "Creattion time: " << creation_time << endl;

    if(abc->getDestAddr() == MACAddress("ff:ff:ff:ff:ff:f0"))
    {
        debugEV << "Match here!!!!" << endl;
    }

    macQueue.pop_front();
    debugEV << "Max queue length: " << queueLength << ", packet put in queue"
                  "\n  queue size: " << macQueue.size() << " macState: "
                  << macState << endl;

    */

    return true;
}

using namespace std;

void BasicRicerMacLayer::attachSignal(macpkt_ptr_t macPkt) //TODO
{  int rand1 =0;
    int rand2 =0;
    if(nodeID >= 0 && nodeID<=4){
     rand1 = rand() % 40 ;
     rand2 = rand() % 40 ;
   }
    else if(nodeID>4 && nodeID<=8){
         rand1 = 60 +( rand()  % 40); ;
         rand2 = rand() % 40; ;
        }
    else if(nodeID>8 && nodeID<=11){
          rand1 = rand() % 20; ;
          rand2 = 60 + (rand() % 40 );
            }
    else if(nodeID>11 && nodeID<=14){
          rand1 = 60 + (rand() % 40); ;
          rand2 = 60 + (rand() % 40); ;
            }
    int rand3 = rand() %100;

    if(macPkt->getKind() == RICER_DATA)
     {//
      debugEV << "********** DATA PACKET SIGNAL ATTACHING **********" << endl;

      ofstream  out;
      out.open("C:/DataSet/CppToJava.txt", std::ios_base::app);
      out << nodeID;
      out << " " ;
      out << rand1;
      out << " " ;
      out << rand2;
      out << " " ;
      out << rand3;
      out << "\n";

     countvect ++;

     debugEV << "********** DATA PACKET SIGNAL ATTACHING : COUNTVECT:" << countvect << endl;

   if(countvect % 50 == 0){

         volatile double slot;
         string line;
         ifstream in ;
         std::string output;
         std::string output1;
         std::string filename ="C:/DataSet/end_device_.txt";


         std::string str = std::to_string(nodeID);
         filename.insert(22,str);

         debugEV << " ***********str : " << str << " ***** filename: " << filename << endl;
         debugEV << "********** DATA PACKET SIGNAL ATTACHING **********" << endl;

         i = slotDuration ;
         std::string chr;

         std::string chr1;

        do{

            in.open(filename, std::ios_base::in);


            if (in.is_open()) {

                debugEV << "********* FILE IS OPEN ************" << endl;

            while (!in.eof()) {
               in >> output >> output1 ;

               debugEV << "********* FROM FILE!!!! :::::::::::: output: " << output << " output1:" << output1 << endl;

            }

           }else {debugEV << "***********  FILE WAS NOT OPEN!!! " << endl;}

           in.close();

           i = stoi(output, 0, 10);

           slotDuration = i;

           slotVector.record(slotDuration);

           }while(slot==i);

          int add = stoi(output1, 0, 10);

          slotVector.record(slotDuration);

          debugEV << "***********  CONVERSION :  " << add << endl;

        i = slot;
        switch(add){



          hexAdd = "00-00-00-00-00-00" + nodeID;

        }

        debugEV << "***********  DEST MAC ADDRESS: " << myMacAddr << endl;

       }
     }

     //create and initialize control info with new signal
    //calc signal duration
    simtime_t duration = macPkt->getBitLength() / bitrate;
    setDownControlInfo(macPkt, createSignal(simTime(), duration, txPower, bitrate));


}


/**
 * Change the color of the node for animation purposes.
 */

void BasicRicerMacLayer::changeDisplayColor(BMAC_COLORS color)
{
    if (!animation)
            return;
        cDisplayString& dispStr = findHost()->getDisplayString();
        //b=40,40,rect,black,black,2"
        if (color == GREEN)
            dispStr.parse("b=15,15,rect,green,green,2");
            //dispStr.parse("bgb=210,450,green;i=device/wifilaptop_s;b=40,40,rect");

            //dispStr.setTagArg("b", 3, "green");

            //dispStr.parse("b=20,20,rect,green,green,2;i=device/WSN_360_Green_vs");

        if (color == BLUE)
            //dispStr.parse("bgb=210,450,blue;i=device/wifilaptop_s;b=40,40,rect");
            //dispStr.setTagArg("b", 3, "blue");
                    dispStr.parse("b=15,15,rect,blue,blue,2");
        if (color == RED)
        {    //dispStr.parse("bgb=210,450,red;i=device/wifilaptop_s;b=40,40,rect");
            //dispStr.setTagArg("b", 3, "red");
             dispStr.parse("b=15,15,rect,red,red,2");
             startIdle = simTime().dbl();
        }
        if (color == BLACK)
            //dispStr.parse("bgb=210,450,black;i=device/wifilaptop_s;b=40,40,rect");
            //dispStr.setTagArg("b", 3, "black");
                    dispStr.parse("b=15,15,rect,black,black,2");
        if (color == YELLOW)
            //dispStr.parse("bgb=210,450,yellow;i=device/wifilaptop_s;b=40,40,rect");
            //dispStr.setTagArg("b", 3, "yellow");
             dispStr.parse("b=15,15,rect,yellow,yellow,2");

}


void BasicRicerMacLayer::sendBeaconPacket()
{
    macpkt_ptr_t beacon = new MacPkt();
    beacon->setSrcAddr(myMacAddr);
    beacon->setDestAddr(LAddress::L2BROADCAST);
    beacon->setKind(RICER_BEACON);
    beacon->setBitLength(16 + headerLength);
    beacon->setSrcProcId(nodeID);
    beacon->setSequenceId(parentHops);

    debugEV << "****** NODE NUM : " << nodeID << " ****** with MAC ADDRESS :" << myMacAddr << endl;

    attachSignal(beacon);
    sendDown(beacon);

}


void BasicRicerMacLayer::sendDataPacket(){


    macpkt_ptr_t data = new MacPkt();

    data->setSrcAddr(myMacAddr);
    data->setDestAddr(MACAddress(hexAdd));
    data->setKind(RICER_DATA);
    data->setBitLength(128 + headerLength);

    data->setSrcProcId(nodeID);
    data->setSequenceId(simTime().dbl() * 1000);
    debugEV << "****Tx: DestAddress is : " << data->getDestAddr() << endl;
    attachSignal(data);
    sendDown(data);
}

void BasicRicerMacLayer::sendAckPacket()
{
    macpkt_ptr_t ack = new MacPkt();
    ack->setSrcAddr(myMacAddr);
    ack->setDestAddr(LAddress::L2BROADCAST);
    ack->setKind(RICER_ACK);
    ack->setBitLength(16 + headerLength);
    ack->setSrcProcId(nodeID);

    //double avgWindSpeed = objPowerManager->predictWindSpeed(objPowerManager->intPredictionPeriod);

    double avgWindPower = 1;//objPowerManager->predictWindPower(objPowerManager->intPredictionPeriod);


    debugEV << "****Rx: Average wind power is: " << avgWindPower << endl;

    ack->setSequenceId((int)(avgWindPower*10));


    attachSignal(ack);
    sendDown(ack);
}




void  BasicRicerMacLayer::forwardPacket(macpkt_ptr_t packet)
{

    macpkt_ptr_t forward = new MacPkt;
    forward->setSrcAddr(packet->getSrcAddr());
    forward->setDestAddr(packet->getDestAddr());
    forward->setSrcProcId(packet->getSrcProcId());
    forward->setSequenceId(packet->getSequenceId());
    forward->setBitLength(packet->getBitLength());
    forward->setKind(RICER_DATA);

    attachSignal(forward);
    sendDown(forward);
}


void BasicRicerMacLayer::generatePacket()
{

    debugEV << " ********* GENERATING PACKET ! ! ************** " << endl;

    //create a packet and add to macQueue
    macpkt_ptr_t data = new MacPkt();
    data->setSrcAddr(myMacAddr);
    data->setDestAddr(MACAddress(hexAdd));
    data->setSrcProcId(nodeID);
    data->setSequenceId(simTime().dbl() * 1000);
    data->setBitLength(128 + headerLength);
    data->setKind(RICER_DATA);
    macQueue.push_back(data);
    nbTxDataPackets++;

}

void  BasicRicerMacLayer::calConsumedEnergy(cMessage* msg)
{



    if(msg->getKind() == TX_FINISH)
    {
        if(macState == Tx_FINISH_DATA)
        {
            totalEnergy += energyDT / (slotDuration + forwarded);
            slotEnergy += energyDT / (slotDuration + forwarded);
        }
        else if(macState == RELAY_FINISH_BEACON)
        {
            totalEnergy += energyWUB / (slotDuration + forwarded);
            slotEnergy += energyWUB / (slotDuration + forwarded);
        }
        else if(macState == RELAY_FINISH_ACK)
        {
            totalEnergy += energyACK / (slotDuration + forwarded);
            slotEnergy += energyACK / (slotDuration + forwarded);
        }
        return;
    }

    debugEV << "**** Cal consumed energy1111 : " << msg->getKind()  << endl;

    if(msg->getKind() != RICER_TIME_OUT && msg->getKind() != RICER_BEACON
    && msg->getKind() != RICER_DATA && msg->getKind()!= RICER_ACK && msg->getKind() != DATA_COLLISION)
        return;


    debugEV << "**** Cal consumed energy : " << msg->getKind()  << endl;
    debugEV << "*** start: " << startIdle << " end: " << simTime().dbl();


    endIdle = simTime().dbl();
    durationIdle = endIdle - startIdle;

    totalEnergy += (durationIdle  * rxPower) / (slotDuration + forwarded );
    slotEnergy += (durationIdle  * rxPower) / (slotDuration + forwarded);

    totalIdle += durationIdle;

    if(msg->getKind() == RICER_BEACON)
    {
        totalEnergy += energyWUB / (slotDuration + forwarded );
        slotEnergy += energyWUB / (slotDuration + forwarded);
    }
    else if(msg->getKind() == RICER_DATA)
    {
        totalEnergy += energyDR / (slotDuration + forwarded) ;
        slotEnergy += energyDR / (slotDuration + forwarded);
    }
    else if(msg->getKind() == RICER_ACK)
    {
        totalEnergy += energyACK / (slotDuration + forwarded);
        slotEnergy += energyACK / (slotDuration + forwarded);
    }
    else if (msg->getKind() == DATA_COLLISION)
    {
        totalEnergy += energyCOL / (slotDuration + forwarded);
        slotEnergy += energyCOL / (slotDuration + forwarded);
    }

    ofstream  out;
                 out.open("C:/DataSet/CppToJavaEnergy.txt", std::ios_base::app);
                 out << simTime() ;
                 out << " ";
                 out << totalEnergy;
                 out << "\n";


      energyState.record(totalEnergy);

}

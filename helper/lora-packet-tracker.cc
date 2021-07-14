/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 University of Padova
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Davide Magrin <magrinda@dei.unipd.it>
 */

#include "lora-packet-tracker.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/lorawan-mac-header.h"
#include <iostream>
#include <fstream>

namespace ns3 {
namespace lorawan {
NS_LOG_COMPONENT_DEFINE ("LoraPacketTracker");

LoraPacketTracker::LoraPacketTracker ()
{
  NS_LOG_FUNCTION (this);
}

LoraPacketTracker::~LoraPacketTracker ()
{
  NS_LOG_FUNCTION (this);
}

/////////////////
// MAC metrics //
/////////////////

void
LoraPacketTracker::MacTransmissionCallback (Ptr<Packet const> packet)
{
  if (IsUplink (packet))
    {
      NS_LOG_INFO ("A new packet was sent by the MAC layer");

      MacPacketStatus status;
      status.packet = packet;
      status.sendTime = Simulator::Now ();
      status.senderId = Simulator::GetContext ();
      status.receivedTime = Time::Max ();

      m_macPacketTracker.insert (std::pair<Ptr<Packet const>, MacPacketStatus>
                                   (packet, status));
    }
}

void
LoraPacketTracker::RequiredTransmissionsCallback (uint8_t reqTx, bool success,
                                                  Time firstAttempt,
                                                  Ptr<Packet> packet)
{
  NS_LOG_INFO ("Finished retransmission attempts for a packet");
  NS_LOG_DEBUG ("Packet: " << packet << "ReqTx " << unsigned(reqTx) <<
                ", succ: " << success << ", firstAttempt: " <<
                firstAttempt.GetSeconds ());

  RetransmissionStatus entry;
  entry.firstAttempt = firstAttempt;
  entry.finishTime = Simulator::Now ();
  entry.reTxAttempts = reqTx;
  entry.successful = success;

  m_reTransmissionTracker.insert (std::pair<Ptr<Packet>, RetransmissionStatus>
                                    (packet, entry));
}

void
LoraPacketTracker::MacGwReceptionCallback (Ptr<Packet const> packet)
{
  if (IsUplink (packet))
    {
      NS_LOG_INFO ("A packet was successfully received" <<
                   " at the MAC layer of gateway " <<
                   Simulator::GetContext ());

      // Find the received packet in the m_macPacketTracker
      auto it = m_macPacketTracker.find (packet);
      if (it != m_macPacketTracker.end ())
        {
          (*it).second.receptionTimes.insert (std::pair<int, Time>
                                                (Simulator::GetContext (),
                                                Simulator::Now ()));
        }
      else
        {
          NS_ABORT_MSG ("Packet not found in tracker");
        }
    }
}

/////////////////
// PHY metrics //
/////////////////

void
LoraPacketTracker::TransmissionCallback (Ptr<Packet const> packet, uint32_t edId)
{
  if (IsUplink (packet))
    {
      NS_LOG_INFO ("PHY packet " << packet
                                 << " was transmitted by device "
                                 << edId);
      // Create a packetStatus
      PacketStatus status;
      status.packet = packet;
      status.sendTime = Simulator::Now ();
      status.senderId = edId;

      m_packetTracker.insert (std::pair<Ptr<Packet const>, PacketStatus> (packet, status));
    }
}

void
LoraPacketTracker::PacketReceptionCallback (Ptr<Packet const> packet, uint32_t gwId)
{
  if (IsUplink (packet))
    {
      // Remove the successfully received packet from the list of sent ones
      NS_LOG_INFO ("PHY packet " << packet
                                 << " was successfully received at gateway "
                                 << gwId);


      if(m_packetTracker.count(packet) == 1) //first reception
        {
          std::map<Ptr<Packet const>, PacketStatus>::iterator it = m_packetTracker.find (packet);
          (*it).second.outcomes.insert (std::pair<int, enum PhyPacketOutcome> (gwId,
                                                                           RECEIVED));
        }
      else
        {
          std::pair <std::multimap<Ptr<Packet const>, PacketStatus>::iterator, std::multimap<Ptr<Packet const>, PacketStatus>::iterator> ret;
          ret = m_packetTracker.equal_range(packet); // find all instances of received packet
          
          int i = 1;

          //loop through all instances and find the one which doesn't yet have an outcome at this gateway
          for(std::multimap<Ptr<Packet const>, PacketStatus>::iterator it=ret.first; it != ret.second; ++it)
            {
                    
              if((*it).second.outcomes.find(gwId) == (*it).second.outcomes.end()) 
              {
                NS_LOG_DEBUG("This is copy  " << i <<" of packet "<< packet);                
                NS_LOG_DEBUG("This packet was sent at " << (*it).second.sendTime);
                NS_LOG_DEBUG("It was not yet received by GW " << gwId << " logging it now as RECEIVED.");
                (*it).second.outcomes.insert(std::pair<int, enum PhyPacketOutcome> (gwId,
                                                                           RECEIVED)); 
              }
              i++;           
            }
        }
    }
}

void
LoraPacketTracker::InterferenceCallback (Ptr<Packet const> packet, uint32_t gwId)
{
  if (IsUplink (packet))
    {
      NS_LOG_INFO ("PHY packet " << packet
                                 << " was interfered at gateway "
                                 << gwId);

      if(m_packetTracker.count(packet) == 1) //first reception
        {
          std::map<Ptr<Packet const>, PacketStatus>::iterator it = m_packetTracker.find (packet);
          (*it).second.outcomes.insert (std::pair<int, enum PhyPacketOutcome> (gwId,
                                                                           INTERFERED)); 
        }
      else
        {
          std::pair <std::multimap<Ptr<Packet const>, PacketStatus>::iterator, std::multimap<Ptr<Packet const>, PacketStatus>::iterator> ret;
          ret = m_packetTracker.equal_range(packet); // find all instances of received packet
          
          int i = 1;

          //loop through all instances and find the one which doesn't yet have an outcome at this gateway
          for(std::multimap<Ptr<Packet const>, PacketStatus>::iterator it=ret.first; it != ret.second; ++it)
            {
                    
              if((*it).second.outcomes.find(gwId) == (*it).second.outcomes.end()) 
              {
                NS_LOG_INFO("This is copy  " << i <<" of packet "<< packet);                
                NS_LOG_INFO("This packet was sent at " << (*it).second.sendTime);
                NS_LOG_INFO("It was not yet received by GW " << gwId << " logging it now as INTERFERED.");
                (*it).second.outcomes.insert(std::pair<int, enum PhyPacketOutcome> (gwId,
                                                                           INTERFERED)); 
              }
              i++;           
            }
        }
    }
}

void
LoraPacketTracker::NoMoreReceiversCallback (Ptr<Packet const> packet, uint32_t gwId)
{
  if (IsUplink (packet))
    {
      NS_LOG_INFO ("PHY packet " << packet
                                 << " was lost because no more receivers at gateway "
                                 << gwId);

      if(m_packetTracker.count(packet) == 1) //first reception
        {
          std::map<Ptr<Packet const>, PacketStatus>::iterator it = m_packetTracker.find (packet);
          (*it).second.outcomes.insert (std::pair<int, enum PhyPacketOutcome> (gwId,
                                                                           NO_MORE_RECEIVERS)); 
        }
      else
        {
          std::pair <std::multimap<Ptr<Packet const>, PacketStatus>::iterator, std::multimap<Ptr<Packet const>, PacketStatus>::iterator> ret;
          ret = m_packetTracker.equal_range(packet); // find all instances of received packet
          
          int i = 1;

          //loop through all instances and find the one which doesn't yet have an outcome at this gateway
          for(std::multimap<Ptr<Packet const>, PacketStatus>::iterator it=ret.first; it != ret.second; ++it)
            {
                    
              if((*it).second.outcomes.find(gwId) == (*it).second.outcomes.end()) 
              {
                NS_LOG_INFO("This is copy  " << i <<" of packet "<< packet);                
                NS_LOG_INFO("This packet was sent at " << (*it).second.sendTime);
                NS_LOG_INFO("It was not yet received by GW " << gwId << " logging it now as NO_MORE_RECEIVERS.");
                (*it).second.outcomes.insert(std::pair<int, enum PhyPacketOutcome> (gwId,
                                                                           NO_MORE_RECEIVERS)); 
              }
              i++;           
            }
        }                                                                     
    }
}

void
LoraPacketTracker::UnderSensitivityCallback (Ptr<Packet const> packet, uint32_t gwId)
{
  if (IsUplink (packet))
    {
      NS_LOG_INFO ("PHY packet " << packet
                                 << " was lost because under sensitivity at gateway "
                                 << gwId);

      if(m_packetTracker.count(packet) == 1) //first reception
        {
          std::map<Ptr<Packet const>, PacketStatus>::iterator it = m_packetTracker.find (packet);
          (*it).second.outcomes.insert (std::pair<int, enum PhyPacketOutcome> (gwId,
                                                                           UNDER_SENSITIVITY)); 
        }
      else
        {
          std::pair <std::multimap<Ptr<Packet const>, PacketStatus>::iterator, std::multimap<Ptr<Packet const>, PacketStatus>::iterator> ret;
          ret = m_packetTracker.equal_range(packet); // find all instances of received packet
          
          int i = 1;

          //loop through all instances and find the one which doesn't yet have an outcome at this gateway
          for(std::multimap<Ptr<Packet const>, PacketStatus>::iterator it=ret.first; it != ret.second; ++it)
            {
                    
              if((*it).second.outcomes.find(gwId) == (*it).second.outcomes.end()) 
              {
                NS_LOG_INFO("This is copy  " << i <<" of packet "<< packet);                
                NS_LOG_INFO("This packet was sent at " << (*it).second.sendTime);
                NS_LOG_INFO("It was not yet received by GW " << gwId << " logging it now as UNDER_SENSITIVITY.");
                (*it).second.outcomes.insert(std::pair<int, enum PhyPacketOutcome> (gwId,
                                                                           UNDER_SENSITIVITY)); 
              }
              i++;           
            }
        }    
    }
}

void
LoraPacketTracker::LostBecauseTxCallback (Ptr<Packet const> packet, uint32_t gwId)
{
  if (IsUplink (packet))
    {
      NS_LOG_INFO ("PHY packet " << packet
                                 << " was lost because of GW transmission at gateway "
                                 << gwId);

      if(m_packetTracker.count(packet) == 1) //first reception
        {
          std::map<Ptr<Packet const>, PacketStatus>::iterator it = m_packetTracker.find (packet);
          (*it).second.outcomes.insert (std::pair<int, enum PhyPacketOutcome> (gwId,
                                                                           LOST_BECAUSE_TX)); 
        }
      else
        {
          std::pair <std::multimap<Ptr<Packet const>, PacketStatus>::iterator, std::multimap<Ptr<Packet const>, PacketStatus>::iterator> ret;
          ret = m_packetTracker.equal_range(packet); // find all instances of received packet
          
          int i = 1;

          //loop through all instances and find the one which doesn't yet have an outcome at this gateway
          for(std::multimap<Ptr<Packet const>, PacketStatus>::iterator it=ret.first; it != ret.second; ++it)
            {
                    
              if((*it).second.outcomes.find(gwId) == (*it).second.outcomes.end()) 
              {
                NS_LOG_INFO("This is copy  " << i <<" of packet "<< packet);                
                NS_LOG_INFO("This packet was sent at " << (*it).second.sendTime);
                NS_LOG_INFO("It was not yet received by GW " << gwId << " logging it now as LOST_BECAUSE_TX.");
                (*it).second.outcomes.insert(std::pair<int, enum PhyPacketOutcome> (gwId,
                                                                           LOST_BECAUSE_TX)); 
              }
              i++;           
            }
        }    
    }
}

bool
LoraPacketTracker::IsUplink (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (this);

  LorawanMacHeader mHdr;
  Ptr<Packet> copy = packet->Copy ();
  copy->RemoveHeader (mHdr);
  return mHdr.IsUplink ();
}

////////////////////////
// Counting Functions //
////////////////////////

std::vector<int>
LoraPacketTracker::CountPhyPacketsPerGw (Time startTime, Time stopTime,
                                         int gwId)
{
  // Vector packetCounts will contain - for the interval given in the input of
  // the function, the following fields: totPacketsSent receivedPackets
  // interferedPackets noMoreGwPackets underSensitivityPackets lostBecauseTxPackets

  std::vector<int> packetCounts (6, 0);

  for (auto itPhy = m_packetTracker.begin ();
       itPhy != m_packetTracker.end ();
       ++itPhy)
    {
      if ((*itPhy).second.sendTime >= startTime && (*itPhy).second.sendTime <= stopTime)
        {
          packetCounts.at (0)++;

          NS_LOG_DEBUG ("Dealing with packet " << (*itPhy).second.packet);
          NS_LOG_DEBUG ("This packet was received by " <<
                        (*itPhy).second.outcomes.size () << " gateways");

          if ((*itPhy).second.outcomes.count (gwId) > 0)
            {
              switch ((*itPhy).second.outcomes.at (gwId))
                {
                case RECEIVED:
                  {
                    packetCounts.at (1)++;
                    break;
                  }
                case INTERFERED:
                  {
                    packetCounts.at (2)++;
                    break;
                  }
                case NO_MORE_RECEIVERS:
                  {
                    packetCounts.at (3)++;
                    break;
                  }
                case UNDER_SENSITIVITY:
                  {
                    packetCounts.at (4)++;
                    break;
                  }
                case LOST_BECAUSE_TX:
                  {
                    packetCounts.at (5)++;
                    break;
                  }
                case UNSET:
                  {
                    break;
                  }
                }
            }
        }
    }

  return packetCounts;
}
std::string
LoraPacketTracker::PrintPhyPacketsPerGw (Time startTime, Time stopTime,
                                         int gwId)
{
  // Vector packetCounts will contain - for the interval given in the input of
  // the function, the following fields: totPacketsSent receivedPackets
  // interferedPackets noMoreGwPackets underSensitivityPackets lostBecauseTxPackets

  std::vector<int> packetCounts (6, 0);

  for (auto itPhy = m_packetTracker.begin ();
       itPhy != m_packetTracker.end ();
       ++itPhy)
    {
      if ((*itPhy).second.sendTime >= startTime && (*itPhy).second.sendTime <= stopTime)
        {
          packetCounts.at (0)++;

          NS_LOG_DEBUG ("Dealing with packet " << (*itPhy).second.packet);
          NS_LOG_DEBUG ("This packet was received by " <<
                        (*itPhy).second.outcomes.size () << " gateways");

          if((*itPhy).second.outcomes.find(gwId) != (*itPhy).second.outcomes.end()) 
            NS_LOG_DEBUG ("Packet outcome:" << (*itPhy).second.outcomes.at(gwId));

          if ((*itPhy).second.outcomes.count (gwId) > 0)
            {
              switch ((*itPhy).second.outcomes.at (gwId))
                {
                case RECEIVED:
                  {
                    packetCounts.at (1)++;
                    break;
                  }
                case INTERFERED:
                  {
                    packetCounts.at (2)++;
                    break;
                  }
                case NO_MORE_RECEIVERS:
                  {
                    packetCounts.at (3)++;
                    break;
                  }
                case UNDER_SENSITIVITY:
                  {
                    packetCounts.at (4)++;
                    break;
                  }
                case LOST_BECAUSE_TX:
                  {
                    packetCounts.at (5)++;
                    break;
                  }
                case UNSET:
                  {
                    break;
                  }
                }
            }
        }
    }

  std::string output ("");
  for (int i = 0; i < 6; ++i)
    {
      output += std::to_string (packetCounts.at (i)) + " ";
    }

  return output;
}

  std::string
  LoraPacketTracker::CountMacPacketsGlobally (Time startTime, Time stopTime)
  {
    NS_LOG_FUNCTION (this << startTime << stopTime);

    double sent = 0;
    double received = 0;
    for (auto it = m_macPacketTracker.begin ();
         it != m_macPacketTracker.end ();
         ++it)
      {
        if ((*it).second.sendTime >= startTime && (*it).second.sendTime <= stopTime)
          {
            sent++;
            if ((*it).second.receptionTimes.size ())
              {
                received++;
              }
          }
      }

    return std::to_string (sent) + " " +
      std::to_string (received);
  }

  std::string
  LoraPacketTracker::CountMacPacketsGloballyCpsr (Time startTime, Time stopTime)
  {
    NS_LOG_FUNCTION (this << startTime << stopTime);

    double sent = 0;
    double received = 0;
    for (auto it = m_reTransmissionTracker.begin ();
         it != m_reTransmissionTracker.end ();
         ++it)
      {
        if ((*it).second.firstAttempt >= startTime && (*it).second.firstAttempt <= stopTime)
          {

            if((*it).first != 0) //Check if Packet isn't 0. 
            //RequiredTransmissionsCallback() frequently has a Packet: 0 call
            {
              sent++;
              NS_LOG_DEBUG ("Found a packet" << (*it).first << ", sent " << (it->second.firstAttempt));
              //NS_LOG_DEBUG ("Number of attempts: " << unsigned(it->second.reTxAttempts) <<
              //             ", successful: " << it->second.successful);
              if (it->second.successful)
                {
                  received++;
                }
              }
          }
      }

    return std::to_string (sent) + " " +
      std::to_string (received);
  }

  void
  LoraPacketTracker::PrintPerformance (Time start, Time stop, int gwId)
  {
    NS_LOG_FUNCTION (this);

    // Statistics ignoring transient
    CountRetransmissionsPorted (start, stop, gwId,0);
  }

  std::string
  LoraPacketTracker::getPerformanceLegend()
  {
     return   "Total unconfirmed Successful unconfirmed | Successfully extracted confirmed packets Successfully ACKed confirmed packets | Incomplete Confirmed | Successful with 1 Successful with 2 Successful with 3 Successful with 4 Successful with 5 Successful with 6 Successful with 7 Successful with 8 | Failed after 1 Failed after 2 Failed after 3 Failed after 4 Failed after 5 Failed after 6 Failed after 7 Failed after 8 | Average Delay Average ACK Delay | Total Retransmission amounts || PHY Total PHY Successful PHY Interfered PHY No More Receivers PHY Under Sensitivity PHY Lost Because TX ** CPSR confirmed sent CPSR confirmed ACKed\n";
  }

 std::string
  LoraPacketTracker::CountRetransmissionsPorted (Time startTime, Time stopTime, int gwId, int returnString)
  {
    std::vector<int> totalReTxAmounts (8, 0);
    std::vector<int> successfulReTxAmounts (8, 0);
    std::vector<int> failedReTxAmounts (8, 0);
    Time delaySum = Seconds (0);
    Time ackDelaySum = Seconds(0);

    int confirmedPacketsOutsideTransient = 0;
    int confirmedMACpacketsOutsideTransient = 0;
    int successfullyExtractedConfirmedPackets = 0;

    int successfulUnconfirmedPackets = 0;
    int incompleteConfirmedPackets = 0;
    int totalUnconfirmedPackets = 0;
    int successfullyAckedConfirmedPackets = 0;

    std::string returnValue="";
  
    for (auto itMac = m_macPacketTracker.begin (); itMac != m_macPacketTracker.end(); ++itMac)
        {


          if ((*itMac).second.sendTime >= startTime && (*itMac).second.sendTime <= stopTime)
            {
              NS_LOG_DEBUG(" ");
              NS_LOG_DEBUG ("Dealing with packet " << (*itMac).first);
              NS_LOG_DEBUG("sendTime " << (*itMac).second.sendTime.GetSeconds());
              NS_LOG_DEBUG("senderId " << (*itMac).second.senderId);
              if((*itMac).second.receptionTimes.size() >0)
                NS_LOG_DEBUG("receptionTimes " << (*itMac).second.receptionTimes.at(gwId).GetSeconds());
              else
              {
                  NS_LOG_DEBUG("Packet sent but never received " << " stopTime was " << stopTime.GetSeconds());
              }
              

              
              // Count retransmissions
              ////////////////////////
              auto itRetx = m_reTransmissionTracker.find ((*itMac).first);
        
              if (itRetx == m_reTransmissionTracker.end() && CheckIfUnconfirmed((*itMac).first))
                {
                  NS_LOG_DEBUG("Packet was a unconfirmed packet");
                  totalUnconfirmedPackets++;

 //                 if ((*itMac).second.receivedTime != Time::Max ()) // Received correctly
                  if ((*itMac).second.receptionTimes.size() != 0) // Received correctly
                    {
                      NS_LOG_DEBUG ("Unconfirmed packet was received");
                      successfulUnconfirmedPackets++;
                      delaySum += ((*itMac).second.receptionTimes.at(gwId)) - (*itMac).second.sendTime;
                    }
                  else
                  {
                         NS_LOG_DEBUG("Unconfirmed packet sent but never received");
                  }
                  
                  // NS_ABORT_MSG ("Searched packet was not found" << "Packet " <<
                  //               (*itMac).first << " not found. Sent at " <<
                  //               (*itMac).second.sendTime.GetSeconds());
                }
              else if(itRetx == m_reTransmissionTracker.end())
                {
                     NS_LOG_DEBUG("Confirmed packet sent but not yet logged in reTransmissionTracker.");
                     NS_LOG_DEBUG("ACK was not yet received/max retrans attempts not yet reached.");
                     incompleteConfirmedPackets++;
                }
              else 
                {
                  NS_LOG_DEBUG("Packet was a confirmed packet");
                  confirmedPacketsOutsideTransient++;
                  confirmedMACpacketsOutsideTransient++;

                  totalReTxAmounts.at ((*itRetx).second.reTxAttempts - 1)++;

                  if ((*itRetx).second.successful)
                    {
                      successfulReTxAmounts.at ((*itRetx).second.reTxAttempts - 1)++;
                      // If this packet was successful at the ED, it means that it
                      // was also received at the GW
                      successfullyExtractedConfirmedPackets++;

                      //Recording to say ED got ACK
                      successfullyAckedConfirmedPackets++;
                    }
                  else
                    {
                      failedReTxAmounts.at ((*itRetx).second.reTxAttempts - 1)++;
                      // Check if, despite failing to get an ACK at the ED, this
                      // packet was received at the GW
                      if ((*itMac).second.receptionTimes.size() != 0)
                        {
                          successfullyExtractedConfirmedPackets++;
                        }
                    }

                  // Compute delays
                  /////////////////
                  if ((*itMac).second.receptionTimes.size() == 0)
                    {
                       NS_LOG_DEBUG ("Confirmed Packet never received, ignoring it");
                      confirmedPacketsOutsideTransient--;
                    }
                  else
                    {
                      delaySum += (*itMac).second.receptionTimes.at(gwId) - (*itMac).second.sendTime;
                      ackDelaySum += (*itRetx).second.finishTime - (*itRetx).second.firstAttempt;
                    }
                }
            }
        }

    // Sum PHY outcomes

    std::string PhyOutcomes = PrintPhyPacketsPerGw(startTime, stopTime, gwId);

    double avgDelay = 0;
    double avgAckDelay = 0;
    if (confirmedPacketsOutsideTransient)
      {
        avgDelay = (delaySum / (confirmedPacketsOutsideTransient +
                                successfulUnconfirmedPackets)).GetSeconds ();
        avgAckDelay = ((ackDelaySum) / confirmedPacketsOutsideTransient).GetSeconds ();
      }
    
    if(returnString)
    {
     returnValue = std::to_string(totalUnconfirmedPackets) + " " + std::to_string(successfulUnconfirmedPackets) + " | ";
     returnValue += std::to_string(successfullyExtractedConfirmedPackets) + " ";
     returnValue += std::to_string(successfullyAckedConfirmedPackets) + " | ";
     returnValue += std::to_string(incompleteConfirmedPackets);
     returnValue += " | ";    
     returnValue += PrintVector (successfulReTxAmounts, 1);
     returnValue +=  " | ";
     returnValue +=  PrintVector (failedReTxAmounts, 1);
     returnValue += " | ";
     returnValue += std::to_string(avgDelay) + " ";
     returnValue += std::to_string(avgAckDelay) + " ";
     returnValue += " | ";
     returnValue += PrintSumRetransmissions (totalReTxAmounts, 1);
     returnValue += " || ";
     returnValue += PhyOutcomes;

     std::string cpsr_result = CountMacPacketsGloballyCpsr(startTime, stopTime);
     returnValue += " ** ";
     returnValue += cpsr_result;
     return returnValue;
    }

    else
    {
      // Print legend
      std::cout << getPerformanceLegend() << std::endl;
      std::cout << totalUnconfirmedPackets << " " << successfulUnconfirmedPackets << " | ";
      std::cout << successfullyExtractedConfirmedPackets << " ";
      std::cout << successfullyAckedConfirmedPackets << " | ";
      std::cout << incompleteConfirmedPackets << " | ";
      PrintVector (successfulReTxAmounts);
      std::cout << " | ";
      PrintVector (failedReTxAmounts);
      std::cout << " | ";
      std::cout << avgDelay << " ";
      std::cout << avgAckDelay << " ";
      std::cout << " | ";
      PrintSumRetransmissions (totalReTxAmounts);
      std::cout << " || ";
      std::cout<< PhyOutcomes;
      std::cout<< " ** ";
      std::string cpsr_result = CountMacPacketsGloballyCpsr(startTime, stopTime);
      std::cout<< cpsr_result;
      std::cout << std::endl;
  
    }
    return "";


  }

  bool LoraPacketTracker::CheckIfUnconfirmed(Ptr<const Packet> packet)
  {
  //Based of ConfirmedMessagesComponent::OnReceivedPacket in network-controller-components.cc

    // Check whether the received packet requires an acknowledgment.
    LorawanMacHeader mHdr;

    Ptr<Packet> myPacket = packet->Copy ();
    myPacket->RemoveHeader (mHdr);

    //NS_LOG_INFO ("Received packet Mac Header: " << mHdr);


    if (mHdr.GetMType () == LorawanMacHeader::CONFIRMED_DATA_UP)
      {
        NS_LOG_DEBUG ("Packet requires confirmation");
        return false;
      }
      else
      {
        return true;
      }
      
  }

  std::string
  LoraPacketTracker::PrintSumRetransmissions (std::vector<int> reTxVector, int returnString)
  {
    int total = 0;

    for (int i = 0; i < int(reTxVector.size ()); i++)
      {
        total += reTxVector[i] * (i + 1);
      }
    if(returnString)
      return std::to_string(total);
    else  
      std::cout << total;

    return "";
  }

  std::string
  LoraPacketTracker::PrintVector (std::vector<int> vector, int returnString)
  {
    std::string returnValue="";

    for (int i = 0; i < int(vector.size ()); i++)
      {
        if(returnString)
          returnValue += std::to_string(vector.at (i)) + " ";
        else       
          std::cout << vector.at (i) << " ";
      }
    return returnValue;

  }

}
}


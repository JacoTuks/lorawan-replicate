/*
 * This script is the first attempt at having congestion monitoring in the NS and has the advanced metrics and tracking of private lorawan repo.
 * Based off advanced-tracking (for tracking) and adr-example for NS setup.
 * This version is for lorawan-replicate version which represents standard LoRaWAN
 * As a result, some features and parameters don't exist when compared to -extended version of this file.
 
 * 
 *  *  How to run and save output in a file (run this from ns-3 directory)
 * ./waf configure --enable-tests --enable-examples
 * ./waf build
 * ./waf --run "congestion-tracking --confirmedPercentage=50 --appPeriod=4959 --simTimeRatio="middle" --RngRun=401" > output.c 2>&1
 */




#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/lora-packet-tracker.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/lora-helper.h"
#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/double.h"
#include "ns3/random-variable-stream.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/command-line.h"
#include "ns3/network-server-helper.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/building-penetration-loss.h"
#include "ns3/building-allocator.h"
#include "ns3/buildings-helper.h"
#include "ns3/forwarder-helper.h"
#include <algorithm>
#include <ctime>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("Congestion-Tracking");

// Network settings
int nDevices = 1200; 
int nGateways = 1;
double radius = 6300;

//The next three will be changed by sem script as needed
int appPeriodSeconds = 44; //Data packets will be sent at this interval
int confirmedPercentage = 50; // % of devices sending confirmed traffic
std::string simTimeRatio = "all"; //by default run the sim for five periods and use middle 3 periods for calcs


double simulationTime = 5*appPeriodSeconds;  // will be overwritten below to account for new appPeriod provided by sem
                              

// Channel model
//Davide's paper only used the log-distance parts (no shadowing) except for specific sims
bool realisticChannelModel = false;

uint8_t basePacketSize = 10;
uint8_t randomPSizeMin = 0;
uint8_t randomPSizeMax = 0;

uint8_t numberOfTransmissions = 8; // The maximum number of transmissions allowed

// Output control
bool print = true; // Save building locations to buildings.txt

int
main (int argc, char *argv[])
{

Packet::EnablePrinting ();

  CommandLine cmd;
  cmd.AddValue ("appPeriod",
                "The period in seconds to be used by periodically transmitting applications",
                appPeriodSeconds);
  cmd.AddValue("confirmedPercentage", "Which percentage of devices should employ confirmed packets", confirmedPercentage);
  cmd.AddValue ("simTimeRatio", "Number of appPeriod multiples over which to calc final results for (string)", simTimeRatio);
  cmd.AddValue("basePacketSize", "Base size for application packets", basePacketSize);
  cmd.AddValue("randomPSizeMin", "Minimum size for randomly sized application packets", randomPSizeMin); 
  cmd.AddValue("randomPSizeMax", "Maximum size for randomly sized application packets", randomPSizeMax); 
  cmd.Parse (argc, argv);


  simulationTime = 5*appPeriodSeconds; //Updated sim time with new value from sem

  // Set up logging
  LogComponentEnable ("Congestion-Tracking", LOG_LEVEL_ALL);
  LogComponentEnable ("PeriodicSender", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraPacketTracker", LOG_LEVEL_ALL);
  
  LogComponentEnableAll (LOG_PREFIX_FUNC);
  LogComponentEnableAll (LOG_PREFIX_NODE);
  LogComponentEnableAll (LOG_PREFIX_TIME);
  /***********
   *  Setup  *
   ***********/

  // Create the time value from the period
  Time appPeriod = Seconds (appPeriodSeconds);

  // Mobility
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator", "rho", DoubleValue (radius),
                                 "X", DoubleValue (0.0), "Y", DoubleValue (0.0));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

 // Config::SetDefault ("ns3::EndDeviceLorawanMac::ACSControl", BooleanValue(enableACS));

  /************************
   *  Create the channel  *
   ************************/

  // Create the lora channel object
  Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
  loss->SetPathLossExponent (3.76);
  loss->SetReference (1, 7.7); //can also use 8.1, 8.1 comes from private lorawan repo's rawcompletenetwork example

  if (realisticChannelModel)
    {
      // Create the correlated shadowing component
      Ptr<CorrelatedShadowingPropagationLossModel> shadowing =
          CreateObject<CorrelatedShadowingPropagationLossModel> ();

      // Aggregate shadowing to the logdistance loss
      loss->SetNext (shadowing);

      // Add the effect to the channel propagation loss
      Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss> ();

      shadowing->SetNext (buildingLoss);
    }

  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

  Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

  /************************
   *  Create the helpers  *
   ************************/

  // Create the LoraPhyHelper
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channel);

  // Create the LorawanMacHelper
  LorawanMacHelper macHelper = LorawanMacHelper ();

  // Create the LoraHelper
  LoraHelper helper = LoraHelper ();
  helper.EnablePacketTracking (); // Output filename
  // helper.EnableSimulationTimePrinting ();

  //Create the NetworkServerHelper
  NetworkServerHelper nsHelper = NetworkServerHelper ();

  //Create the ForwarderHelper
  ForwarderHelper forHelper = ForwarderHelper ();

  /************************
   *  Create End Devices  *
   ************************/

  // Create a set of nodes
  NodeContainer endDevices;
  endDevices.Create (nDevices);

  // Assign a mobility model to each node
  mobility.Install (endDevices);

  // Make it so that nodes are at a certain height > 0
  for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    {
      Ptr<MobilityModel> mobility = (*j)->GetObject<MobilityModel> ();
      Vector position = mobility->GetPosition ();
      position.z = 1.2;
      mobility->SetPosition (position);
    }

  // Create the LoraNetDevices of the end devices
  uint8_t nwkId = 54;
  uint32_t nwkAddr = 1864;
  Ptr<LoraDeviceAddressGenerator> addrGen =
      CreateObject<LoraDeviceAddressGenerator> (nwkId, nwkAddr);

  // Create the LoraNetDevices of the end devices
  macHelper.SetAddressGenerator (addrGen);
  macHelper.SetRegion (LorawanMacHelper::EU);
  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetDeviceType (LorawanMacHelper::ED_A);
  helper.Install (phyHelper, macHelper, endDevices);

  // Now end devices are connected to the channel

  // Make traffic confirmed if needed

  // Figure out how many devices should employ confirmed traffic
  int confirmedNumber = confirmedPercentage * endDevices.GetN () / 100;
  int i = 0;

  for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    {
      Ptr<Node> node = *j;

      // Set message type (default is unconfirmed)
      Ptr<LorawanMac> edMac =node->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac ();
      Ptr<ClassAEndDeviceLorawanMac> edLorawanMac = edMac->GetObject<ClassAEndDeviceLorawanMac> ();

      // Set message type, otherwise the NS does not send ACKs
      if (i < confirmedNumber)
      {
       edLorawanMac->SetMType (LorawanMacHeader::CONFIRMED_DATA_UP);
      }

      edLorawanMac->SetMaxNumberOfTransmissions (numberOfTransmissions);
      i++;
    }
  


  /*********************
   *  Create Gateways  *
   *********************/

  // Create the gateway nodes (allocate them uniformely on the disc)
  NodeContainer gateways;
  gateways.Create (nGateways);

  Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
  // Make it so that nodes are at a certain height > 0
  allocator->Add (Vector (0.0, 0.0, 15.0));
  mobility.SetPositionAllocator (allocator);
  mobility.Install (gateways);

  // Create a netdevice for each gateway
  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LorawanMacHelper::GW);
  helper.Install (phyHelper, macHelper, gateways);

  /**********************
   *  Handle buildings  *
   **********************/

  double xLength = 130;
  double deltaX = 32;
  double yLength = 64;
  double deltaY = 17;
  int gridWidth = 2 * radius / (xLength + deltaX);
  int gridHeight = 2 * radius / (yLength + deltaY);
  if (realisticChannelModel == false)
    {
      gridWidth = 0;
      gridHeight = 0;
    }
  Ptr<GridBuildingAllocator> gridBuildingAllocator;
  gridBuildingAllocator = CreateObject<GridBuildingAllocator> ();
  gridBuildingAllocator->SetAttribute ("GridWidth", UintegerValue (gridWidth));
  gridBuildingAllocator->SetAttribute ("LengthX", DoubleValue (xLength));
  gridBuildingAllocator->SetAttribute ("LengthY", DoubleValue (yLength));
  gridBuildingAllocator->SetAttribute ("DeltaX", DoubleValue (deltaX));
  gridBuildingAllocator->SetAttribute ("DeltaY", DoubleValue (deltaY));
  gridBuildingAllocator->SetAttribute ("Height", DoubleValue (6));
  gridBuildingAllocator->SetBuildingAttribute ("NRoomsX", UintegerValue (2));
  gridBuildingAllocator->SetBuildingAttribute ("NRoomsY", UintegerValue (4));
  gridBuildingAllocator->SetBuildingAttribute ("NFloors", UintegerValue (2));
  gridBuildingAllocator->SetAttribute (
      "MinX", DoubleValue (-gridWidth * (xLength + deltaX) / 2 + deltaX / 2));
  gridBuildingAllocator->SetAttribute (
      "MinY", DoubleValue (-gridHeight * (yLength + deltaY) / 2 + deltaY / 2));
  BuildingContainer bContainer = gridBuildingAllocator->Create (gridWidth * gridHeight);

  BuildingsHelper::Install (endDevices);
  BuildingsHelper::Install (gateways);

  // Print the buildings
  if (print)
    {
      std::ofstream myfile;
      myfile.open ("buildings.txt");
      std::vector<Ptr<Building>>::const_iterator it;
      int j = 1;
      for (it = bContainer.Begin (); it != bContainer.End (); ++it, ++j)
        {  
          Box boundaries = (*it)->GetBoundaries ();
          myfile << "set object " << j << " rect from " << boundaries.xMin << "," << boundaries.yMin
                 << " to " << boundaries.xMax << "," << boundaries.yMax << std::endl;
        }
      myfile.close ();
    }

  /**********************************************
   *  Set up the end device's spreading factor  *
   **********************************************/

  //Calcultes the lowest SF for each device to use whilst ensuring connectivity.
  macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);

  NS_LOG_DEBUG ("Completed configuration, there are "
                 << endDevices.GetN() << " devices of which "
                 << confirmedNumber << " are confirmed.");

  /*********************************************
   *  Install applications on the end devices  *
   *********************************************/

  Time appStopTime = Seconds (simulationTime);
  PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
  appHelper.SetPeriod (Seconds (appPeriodSeconds));


  appHelper.SetPacketSize(basePacketSize);
  Ptr<RandomVariableStream> rv = CreateObjectWithAttributes<UniformRandomVariable> (
      "Min", DoubleValue (randomPSizeMin), "Max", DoubleValue (randomPSizeMax)); //randomPSizeMin and randomPSizeMax included

  appHelper.SetPacketSizeRandomVariable(rv); //As indicated by email from Martina

  ApplicationContainer appContainer = appHelper.Install (endDevices);

  appContainer.Start (Seconds (0));
  appContainer.Stop (appStopTime);

  /**************************
   *  Create Network Server  *
   ***************************/

  // Create the NS node
  NodeContainer networkServer;
  networkServer.Create (1);

  // Create a NS for the network
  nsHelper.SetEndDevices (endDevices);
  nsHelper.SetGateways (gateways);
  nsHelper.Install (networkServer);

  //Create a forwarder for each gateway
  forHelper.Install (gateways);

  ////////////////
  // Simulation //
  ////////////////

  Simulator::Stop (appStopTime + Seconds (1));

  NS_LOG_INFO ("Running simulation...");
  Simulator::Run ();

  Simulator::Destroy ();

  ///////////////////////////
  // Print results to file //
  ///////////////////////////
  NS_LOG_INFO ("Computing performance metrics...");

  //Want to compute using the middle time section of each simulation.
  //Each sim ran for 5*appPeriodSeconds, so computation must use the middle interval
  //PrintPhyPacketsPerGw can take a start and stop time
  LoraPacketTracker &tracker = helper.GetPacketTracker ();
  //nDevices is the id of the gw as 1200 devices would be 0-1199 with gw being device 1200


  if(simTimeRatio == "middle")
  {
    NS_LOG_INFO ("Computing over the period "<< 1*appPeriodSeconds<< "s to "<< Seconds(2*appPeriodSeconds).GetSeconds() << "s");
    tracker.PrintPerformance(Seconds (appPeriodSeconds), Seconds(2*appPeriodSeconds), nDevices); //option for middle
  }

  else  if(simTimeRatio == "all")
  {
     NS_LOG_INFO ("Computing over the period "<< 1*appPeriodSeconds<< "s to "<< Seconds(4*appPeriodSeconds).GetSeconds() << "s");
    tracker.PrintPerformance(Seconds(appPeriodSeconds), Seconds(4*appPeriodSeconds), nDevices); //option for all 3
  }
  else  if(simTimeRatio == "first")
  {
     NS_LOG_INFO ("Computing over the period "<< Seconds(0)<< "s to "<< Seconds(1*appPeriodSeconds).GetSeconds() << "s");
    tracker.PrintPerformance(Seconds (Seconds(0)), Seconds(1*appPeriodSeconds), nDevices); //option for all 3
  }
  else  if(simTimeRatio == "last")
  {
     NS_LOG_INFO ("Computing over the period "<< Seconds(2*appPeriodSeconds).GetSeconds()<< "s to "<< Seconds(3*appPeriodSeconds).GetSeconds() << "s");
    tracker.PrintPerformance(Seconds (Seconds(2*appPeriodSeconds).GetSeconds()), Seconds(3*appPeriodSeconds), nDevices); //option for all 3
  }
  else if(simTimeRatio == "1h")
  {
    NS_LOG_INFO ("Computing over the period 0s "<< "s to "<< Seconds(6*appPeriodSeconds).GetSeconds() << "s");
    tracker.PrintPerformance(Seconds (0), Seconds(6*appPeriodSeconds), nDevices); //option for middle
  }
  else  if(simTimeRatio == "all10")
  {
     NS_LOG_INFO ("Computing over the period "<< 1*appPeriodSeconds<< "s to "<< Seconds(11*appPeriodSeconds).GetSeconds() << "s");
    tracker.PrintPerformance(Seconds(appPeriodSeconds), Seconds(11*appPeriodSeconds), nDevices); //option for all 3
  }
  else
    {
    NS_LOG_INFO ("Expand .cc file to handle your custom period");
  }
    
  return 0;
}
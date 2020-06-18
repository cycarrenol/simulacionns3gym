#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/mobility-model.h"
#include "ns3/olsr-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/core-module.h"
#include "ns3/opengym-module.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "utility"


using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiVariableAdhocGridWithOpenGym");

const uint32_t NUM_CHANNELS = 4;
uint32_t selected_channel, stepCount, channelCounter[NUM_CHANNELS];


float EXPECTED_PERCENTAGES[NUM_CHANNELS] = { 0.25, 0.25, 0.25, 0.25 };
float percentage[NUM_CHANNELS];

/*
Define observation space
*/
Ptr<OpenGymSpace> MyGetObservationSpace()
{
  float low = 0.0;
  float high = 1.0;
  std::vector<uint32_t> shape = {NUM_CHANNELS,};
  std::string dtype = TypeNameGet<float> ();
  Ptr<OpenGymBoxSpace> space = CreateObject<OpenGymBoxSpace> (low, high, shape, dtype);
  NS_LOG_UNCOND ("MyGetObservationSpace: " << space);
  return space;
}

/*
Define action space
*/
Ptr<OpenGymSpace> MyGetActionSpace()
{
  Ptr<OpenGymDiscreteSpace> space = CreateObject<OpenGymDiscreteSpace> (NUM_CHANNELS);
  NS_LOG_UNCOND ("MyGetActionSpace: " << space);
  return space;
}

/*
Define game over condition
*/
bool MyGetGameOver(void)
{
  uint32_t total = 0;
  for( uint32_t c = 0; c < NUM_CHANNELS; ++ c )
    total += channelCounter[c];

  float diff = 0.0;

  for( uint32_t c = 0; c < NUM_CHANNELS; ++ c )
    diff += fabs( EXPECTED_PERCENTAGES[c] - percentage[c] );

  return total > 100 && diff > 0.5;
}


/*
Collect observations
*/
Ptr<OpenGymDataContainer> MyGetObservation()
{
  std::vector<uint32_t> shape = {NUM_CHANNELS,};
  Ptr<OpenGymBoxContainer<float> > box = CreateObject<OpenGymBoxContainer<float> >(shape);

  for( uint32_t c = 0; c < NUM_CHANNELS; ++ c )
    box->AddValue(percentage[c]);

  NS_LOG_UNCOND ("MyGetObservation: " << box);
  return box;
}


/*
Define reward function
*/
float MyGetReward(void)
{
  float prevDiff = 0.0;   
  for( uint32_t c = 0; c < NUM_CHANNELS; ++ c )
  {
    float prevAvg = 0.0;
    if( c == selected_channel )
      prevAvg = 1.0 * ( channelCounter[c] - 1 ) / ( stepCount - 1 );
    else
      prevAvg = 1.0 * ( channelCounter[c] ) / ( stepCount - 1 );

    prevDiff += fabs( EXPECTED_PERCENTAGES[c] - prevAvg );
  }

  float diff = 0.0;
  for( uint32_t c = 0; c < 4; ++ c )
    diff += fabs( EXPECTED_PERCENTAGES[c] - percentage[c] );

  if( diff < prevDiff ) return 1.0;
  return 0.0;
}

/*
Execute received actions
*/

bool MyExecuteActions(Ptr<OpenGymDataContainer> action)
{      
  Ptr<OpenGymDiscreteContainer> discrete = DynamicCast<OpenGymDiscreteContainer>(action);
  selected_channel = discrete->GetValue();
  ++channelCounter[selected_channel];
  ++stepCount;

  for( uint32_t c = 0; c < NUM_CHANNELS; ++ c )
    percentage[c] = 1.0 * channelCounter[c] / stepCount;

  NS_LOG_UNCOND ("MyExecuteActions: The selected channel is " << selected_channel );
  return true;
}

void ScheduleNextStateRead(double envStepTime, Ptr<OpenGymInterface> openGym)
{
  Simulator::Schedule (Seconds(envStepTime), &ScheduleNextStateRead, envStepTime, openGym);
  openGym->NotifyCurrentState();
}


void ReceivePacket (Ptr<Socket> socket)
{
  while (socket->Recv ())
    {
      NS_LOG_UNCOND ("Received one packet!");
    }
}

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic,
                           socket, pktSize,pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

//
// This program configures a grid (default 5x5) of nodes on an
// 802.11b physical layer, with
// 802.11b NICs in adhoc mode, and by default, sends one packet of 1000
// (application) bytes to node 1.
//
// The default layout is like this, on a 2-D grid.
//
// n20  n21  n22  n23  n24
// n15  n16  n17  n18  n19
// n10  n11  n12  n13  n14
// n5   n6   n7   n8   n9
// n0   n1   n2   n3   n4
//

int
main (int argc, char *argv[]){
  std::string phyMode ("DsssRate1Mbps");
  double distance = 500;
  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 100;
  uint32_t numNodes = 25;

  uint32_t sinkNode = 0; 
  uint32_t sourceNode = 24;
  double interval = 1.0; // seconds
  bool verbose = false;
  bool tracing = true;
  
  //srand(time(NULL));
  // Parameters of the scenario
  uint32_t simSeed = 1;
  double simulationTime = 200; //seconds
  double envStepTime = 0.1; //seconds, ns3gym env step time interval
  uint32_t openGymPort = 5555;
  uint32_t gymArg = 0;

  CommandLine cmd;
  // required parameters for OpenGym interface
  cmd.AddValue ("openGymPort", "Port number for OpenGym env. Default: 5555", openGymPort);
  cmd.AddValue ("simSeed", "Seed for random generator. Default: 1", simSeed);
  // optional parameters
  cmd.AddValue ("simTime", "Simulation time in seconds. Default: 10s", simulationTime);
  cmd.AddValue ("gymArg", "Extra simulation argument. Default: 0", gymArg);

  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("tracing", "turn on ascii and pcap tracing", tracing);
  cmd.AddValue ("numNodes", "number of nodes", numNodes);
  cmd.AddValue ("sinkNode", "Receiver node number", sinkNode);
  cmd.AddValue ("sourceNode", "Sender node number", sourceNode);
  cmd.Parse (argc, argv);
  // Convert to time object
  Time interPacketInterval = Seconds (interval);

  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
                      StringValue (phyMode));

  NS_LOG_UNCOND("Ns3Env parameters:");
  NS_LOG_UNCOND("--simulationTime: " << simulationTime);
  NS_LOG_UNCOND("--openGymPort: " << openGymPort);
  NS_LOG_UNCOND("--envStepTime: " << envStepTime);
  NS_LOG_UNCOND("--seed: " << simSeed);
  NS_LOG_UNCOND("--gymArg: " << gymArg);

  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (simSeed);
  
  NodeContainer nodeContainer;
  nodeContainer.Create (numNodes);

  // The below set of helpers will help us to put together the wifi NICs we want
  WifiHelper wifi;
  if (verbose)
    {
      wifi.EnableLogComponents ();  // Turn on all Wifi logging
    }
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  // set it to zero; otherwise, gain will be added
  wifiPhy.Set ("RxGain", DoubleValue (-10));
  // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);

  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Add an upper mac and disable rate control
  WifiMacHelper wifiMac;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));
  // Set it to adhoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, nodeContainer); 

  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (distance),
                                 "DeltaY", DoubleValue (distance),
                                 "GridWidth", UintegerValue (5),
                                 "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (nodeContainer);

  // Enable OLSR
  OlsrHelper olsr;
  Ipv4StaticRoutingHelper staticRouting;

  Ipv4ListRoutingHelper list;
  list.Add (staticRouting, 0);
  list.Add (olsr, 10);

  InternetStackHelper internet;
  internet.SetRoutingHelper (list); // has effect on the next Install ()
  internet.Install (nodeContainer);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

  // set the nodes different to sourceNode as sink nodes
  for(uint32_t s = 0 ; s <numNodes;s++){
    if(s == sourceNode)continue;
    Ptr<Socket> recvSink = Socket::CreateSocket (nodeContainer.Get (s), tid);
    InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
    recvSink->Bind (local);
    recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));
  }

  Ptr<Socket> recvSink = Socket::CreateSocket (nodeContainer.Get (sinkNode), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

  Ptr<Socket> source = Socket::CreateSocket (nodeContainer.Get (sourceNode), tid);
  InetSocketAddress remote = InetSocketAddress (i.GetAddress (sinkNode, 0), 80);
  source->Connect (remote);

 // Trace
 AsciiTraceHelper ascii;
 wifiPhy.EnableAsciiAll (ascii.CreateFileStream ("sim.tr"));
 wifiPhy.EnablePcap ("simNode", devices);
 // Trace routing tables
 Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("sim.routes", std::ios::out);
 olsr.PrintRoutingTableAllEvery (Seconds (2), routingStream);
 Ptr<OutputStreamWrapper> neighborStream = Create<OutputStreamWrapper> ("sim.neighbors", std::ios::out);
 olsr.PrintNeighborCacheAllEvery (Seconds (2), neighborStream);

  // OpenGym Env  
  Ptr<OpenGymInterface> openGym = CreateObject<OpenGymInterface> (openGymPort);
  openGym->SetGetActionSpaceCb( MakeCallback (&MyGetActionSpace) );
  openGym->SetGetObservationSpaceCb( MakeCallback (&MyGetObservationSpace) );
  openGym->SetGetGameOverCb( MakeCallback (&MyGetGameOver) );
  openGym->SetGetObservationCb( MakeCallback (&MyGetObservation) );
  openGym->SetGetRewardCb( MakeCallback (&MyGetReward) );
  openGym->SetExecuteActionsCb( MakeCallback (&MyExecuteActions) );  
  Simulator::Schedule (Seconds(0.0), &ScheduleNextStateRead, envStepTime, openGym);

  // Give OLSR time to converge-- 30 seconds perhaps
  Simulator::Schedule (Seconds (30.0), &GenerateTraffic, source, packetSize, numPackets, interPacketInterval);

  // Output what we are doing
  NS_LOG_UNCOND ("Testing from node " << sourceNode << " to " << sinkNode << " with grid distance " << distance);

  Simulator::Stop (Seconds (simulationTime));
  Simulator::Run ();
  openGym->NotifySimulationEnd();
  Simulator::Destroy ();
}

/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/automotive-module.h"
#include "ns3/traci-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/lte-helper.h"
#include "ns3/config-store.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/sumo_xml_parser.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/vehicle-visualizer-module.h"
#include "ns3/PRRSupervisor.h"
#include "ns3/three-gpp-spectrum-propagation-loss-model.h"
#include "ns3/oh-buildings-propagation-loss-model.h"
#include "ns3/three-gpp-channel-model.h"
#include <unistd.h>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("lte-v2x");

int
main (int argc, char *argv[])
{
  /*
   * In this example the generated vehicles will send their CAMs toward their LTE Uu interface. The messages will be directed
   * to a server connected to the EPC of the LTE network. In this case, as opposed to 802.11p, it is not possible
   * to broadcast the V2X messages, that will instead be encapsulated inside UDP-IPv4. Thus, each message will have the
   * following encapsulation: BTP+GeoNet+UDP+IPv4+{LTE}. Due to the impossibility of broadcasting messages, also the
   * application logic changes a bit (with respect to 802.11p).
   * In this case, the server computes (at the beginning of the simulation) two areas: the inner area, where the maximum
   * speed is meant to be 25 km/h, and an outer area, where the maximum speed is meant to be 75 km/h. The server monitors
   * the position of the vehicles by reading the CAMs information. As soon as a vehicle performs a transition between the
   * two areas, the server will generate a DENM including an optional container (à la carte), where there is a field named
   * RoadWorks->SpeedLimit that is used to tell the vehicle the maximum allowed speed.
   */

  std::string sumo_folder = "src/automotive/examples/sumo_v2x_map/";
  std::string mob_trace = "map.rou.xml";
  std::string sumo_config ="src/automotive/examples/sumo_v2x_map/testmap.sumocfg";

  /*** a. Environment Options ***/
  double frequency = 3.0e9;
  //the real relative boundaries in the *net.xml file
  double boundary[] = {0.00, 0.00,1629.91,1948.73};
  

  /*** b. App Options ***/
  bool verbose = true;
  bool sumo_gui = true;
  bool aggregate_out = false;
  double sumo_updates = 0.01;
  std::string csv_name;
  std::string csv_name_cumulative;
  std::string sumo_netstate_file_name;
  bool print_summary = false;

  /*** 0.b LENA Options ***/
  double interpacket_interval = 100;

  int num_nodes;
  uint32_t nodeCounter = 0;
  xmlDocPtr rou_xml_file;
  bool vehicle_vis = true;

  // Disabling this option turns off the whole V2X application (useful for comparing the situation when the application is enabled and the one in which it is disabled)
  bool send_cam = true;
  bool send_denm = true;
  double m_baseline_prr = 150.0;
  bool m_prr_sup = false;
  double sim_seconds = 100;


  /* Cmd Line option for vehicular application */
  CommandLine cmd;

  cmd.AddValue ("sumo-gui", "Use SUMO gui or not", sumo_gui);
  cmd.AddValue ("server-aggregate-output", "Print an aggregate output for server", aggregate_out);
  cmd.AddValue ("sumo-updates", "SUMO granularity", sumo_updates);
  cmd.AddValue ("sumo-folder","Position of sumo config files",sumo_folder);
  cmd.AddValue ("mob-trace", "Name of the mobility trace file", mob_trace);
  cmd.AddValue ("sumo-config", "Location and name of SUMO configuration file", sumo_config);
  cmd.AddValue ("csv-log", "Name of the CSV log file", csv_name);
  cmd.AddValue ("vehicle-visualizer", "Activate the web-based vehicle visualizer for ms-van3t", vehicle_vis);
  cmd.AddValue ("send-cam", "Turn on or off the transmission of CAMs, thus turning on or off the whole V2X application",send_cam);
  cmd.AddValue ("csv-log-cumulative", "Name of the CSV log file for the cumulative (average) PRR and latency data", csv_name_cumulative);
  cmd.AddValue ("netstate-dump-file", "Name of the SUMO netstate-dump file containing the vehicle-related information throughout the whole simulation", sumo_netstate_file_name);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("prr-sup","Use the PRR supervisor or not",m_prr_sup);
  cmd.AddValue ("send-cam", "To trigger the CAM dissemination", send_cam);
  cmd.AddValue ("send-denm", "To trigger the DENM dissemination", send_denm);

  /* Cmd Line option for Lena */
  cmd.AddValue("interpacket_interval", "Inter packet interval [ms]", interpacket_interval);
  cmd.AddValue("sim-time", "Total duration of the simulation [s]", sim_seconds);

  cmd.Parse (argc, argv);

  if (verbose)
    {
      LogComponentEnable ("lte-v2x", LOG_LEVEL_INFO);
      LogComponentEnable ("CABasicService", LOG_LEVEL_INFO);
      LogComponentEnable ("DENBasicService", LOG_LEVEL_INFO);
    }


  /*** 0.b Read from the mob_trace the number of vehicles that will be created.
   *       The number of vehicles is directly parsed from the rou.xml file, looking at all
   *       the valid XML elements of type <vehicle>
  ***/
  NS_LOG_INFO("Reading the .rou.xml file...");
  std::string path = sumo_folder + mob_trace;

  /* Load the .rou.xml document */
  xmlInitParser();
  rou_xml_file = xmlParseFile(path.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path);
    }
  /*the number of vehicles(UEs)*/
  num_nodes = XML_rou_count_vehicles(rou_xml_file);

  xmlFreeDoc(rou_xml_file);
  xmlCleanupParser();

  NS_LOG_INFO("The .rou file has been read: " << num_nodes << " vehicles will be present in the simulation.");

  /* Set the simulation time (in seconds) */
  NS_LOG_INFO("Simulation will last " << sim_seconds << " seconds");
  ns3::Time sim_time (ns3::Seconds(sim_seconds));

  /*** Create LTE objects
       the network topology created is the following:

       UEs->(LTE CHANNEL)->enB->(SGW->PGW)->RemoteHost
                                  ^EPC^
   ***/
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(320));
  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults();

  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  /*** Create the remotehost that will gather the CAM and send the DENM ***/
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remotehost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);
  NS_LOG_INFO("remotehost set up");

  /*** Create the p2p connection between the remotehost and pgw ***/
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("10Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.005)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remotehost);

  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("10.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internet_ifaces = ipv4h.Assign (internetDevices);
  /* interface 0 is localhost, 1 is the remotehost*/
  Ipv4Address remotehost_addr = internet_ifaces.GetAddress (1);

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remotehost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);
  NS_LOG_INFO("p2p connection set up");

  /***  Create containers for UEs and eNB ***/
  NodeContainer ue_nodes;
  NodeContainer enb_nodes;
  enb_nodes.Create(1);
  ue_nodes.Create(num_nodes);

  /*** Create and install mobility (SUMO will be attached later) ***/
  MobilityHelper mobility;
  mobility.Install(enb_nodes);
  mobility.Install(ue_nodes);

  /* Set the eNB to a fixed position */
  Ptr<MobilityModel> mobility_enb = enb_nodes.Get (0)->GetObject<MobilityModel> ();
  mobility_enb->SetPosition (Vector ((boundary[0]+boundary[2])/2, (boundary[1]+boundary[3])/2, 20.0)); 
  NetDeviceContainer enb_devs = lteHelper->InstallEnbDevice (enb_nodes);
  NetDeviceContainer ue_devs = lteHelper->InstallUeDevice (ue_nodes);

  /* Install the IP stack on the UEs */
  internet.Install (ue_nodes);
  Ipv4InterfaceContainer ueIpIface;

  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ue_devs));
  NS_LOG_INFO("internet devs installed");
  

  /* Assign IP address to UEs */
  for (uint32_t i = 0; i < ue_nodes.GetN (); i++)
    {
      Ptr<Node> ueNode = ue_nodes.Get (i);
      /* Set the default gateway for the UE */
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

  for (uint16_t i = 0; i < num_nodes; i++)
      {
        lteHelper->Attach (ue_devs.Get(i), enb_devs.Get(0));
        /* side effect: the default EPS bearer will be activated */
      }

  /*** Set attributes for the channel ***/
  /* Channel Condition Settings (Add additional info for the channel) */

  Ptr<ThreeGppChannelModel> channel_model = CreateObject<ThreeGppChannelModel> ();
  channel_model->SetAttribute ("Frequency", DoubleValue (frequency));

  //PathLoss Settings 大尺度衰落
  lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::OhBuildingsPropagationLossModel"));

  //Spectrum Fading Settings 小尺度衰落
  lteHelper->SetAttribute ("FadingModel", StringValue ("ns3::ThreeGppSpectrumPropagationLossModel"));
  lteHelper->SetFadingModelAttribute ("ChannelModel", PointerValue (channel_model));
  NS_LOG_INFO("channel set up");

  /*** Setup Traci and start SUMO ***/
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (sumo_updates)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", (BooleanValue) sumo_gui);
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));
  std::string sumo_additional_options = "--verbose true";

  if(sumo_netstate_file_name!="")
  {
    sumo_additional_options += " --netstate-dump " + sumo_netstate_file_name;
  }

  sumo_additional_options += " --collision.action warn --collision.check-junctions --error-log=sumo-errors-or-collisions.xml";

  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));
  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue (sumo_additional_options));


  /* Create and setup the web-based vehicle visualizer of ms-van3t */
  vehicleVisualizer vehicleVisObj;
  Ptr<vehicleVisualizer> vehicleVis = &vehicleVisObj;
  if (vehicle_vis)
  {
      vehicleVis->startServer();
      vehicleVis->connectToServer ();
      sumoClient->SetAttribute ("VehicleVisualizer", PointerValue (vehicleVis));
  }
  NS_LOG_INFO("visualizer set up");

  Ptr<PRRSupervisor> prrSup = NULL;
  PRRSupervisor prrSupObj(m_baseline_prr);
  if(m_prr_sup)
    {
      prrSup = &prrSupObj;
      prrSup->setTraCIClient(sumoClient);
    }

  /***  Create and Setup application for the server in remotehost ***/
  areaSpeedAdvisorServerLTEHelper V2x_ServerHelper;
  V2x_ServerHelper.SetAttribute ("Client", (PointerValue) sumoClient);
  V2x_ServerHelper.SetAttribute ("AggregateOutput", BooleanValue(aggregate_out));
  V2x_ServerHelper.SetAttribute ("CSV", StringValue(csv_name));
  V2x_ServerHelper.SetAttribute ("PRRSupervisor", PointerValue (prrSup));

  ApplicationContainer AppServer = V2x_ServerHelper.Install (remoteHostContainer.Get (0));

  AppServer.Start (Seconds (0.0));
  AppServer.Stop (sim_time - Seconds (0.1));


  /*** Setup interface and application for dynamic nodes ***/
  areaSpeedAdvisorClientLTEHelper V2x_ClientHelper;
  V2x_ClientHelper.SetAttribute ("ServerAddr", Ipv4AddressValue(remotehost_addr));
  V2x_ClientHelper.SetAttribute ("Client", (PointerValue) sumoClient); // pass TraciClient object for accessing sumo in application
  V2x_ClientHelper.SetAttribute ("PrintSummary", BooleanValue(print_summary));
  V2x_ClientHelper.SetAttribute ("CSV", StringValue(csv_name));
  V2x_ClientHelper.SetAttribute ("PRRSupervisor", PointerValue (prrSup));

  /* callback function for node creation */
  std::function<Ptr<Node> ()> setupNewWifiNode = [&] () -> Ptr<Node>
    {
      if (nodeCounter >= ue_nodes.GetN())
        NS_FATAL_ERROR("Node Pool empty!: " << nodeCounter << " nodes created.");
      std::cout<<"Creating node: "<<nodeCounter<<std::endl;

      /* Don't create and install the protocol stack of the node at simulation time -> take from "node pool" */
      Ptr<Node> includedNode = ue_nodes.Get(nodeCounter);
      ++nodeCounter; // increment counter for next node

      /* Install Application */
      ApplicationContainer ClientApp = V2x_ClientHelper.Install (includedNode);
      ClientApp.Start (Seconds (0.0));
      ClientApp.Stop (sim_time - Simulator::Now () - Seconds (0.1));
      std::cout<<"Setting node done: "<<nodeCounter<<std::endl;

      return includedNode;
    };

  /* Callback function for node shutdown */
  std::function<void (Ptr<Node>)> shutdownWifiNode = [] (Ptr<Node> exNode)
    {
      /* Stop all applications */
      Ptr<areaSpeedAdvisorClientLTE> appClient_ = exNode->GetApplication(0)->GetObject<areaSpeedAdvisorClientLTE>();

      if(appClient_)
        appClient_->StopApplicationNow ();

       /* Set position outside communication range */
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0));// rand() for visualization purposes
      std::cout<<"Shutting Down Node: "<<std::endl;
      /* NOTE: further actions could be required for a safe shut down! */
    };

  /* start traci client with given function pointers */
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  /* enable traces for lte module */
  lteHelper->EnableRlcTraces();

  /*** 9. Start Simulation ***/
  Simulator::Stop (sim_time);

  Simulator::Run ();
  Simulator::Destroy ();

  // if(m_prr_sup)
  //   {
  //     if(csv_name_cumulative!="")
  //     {
  //       std::ofstream csv_cum_ofstream;
  //       std::string full_csv_name = csv_name_cumulative + ".csv";

  //       if(access(full_csv_name.c_str(),F_OK)!=-1)
  //       {
  //         // The file already exists
  //         csv_cum_ofstream.open(full_csv_name,std::ofstream::out | std::ofstream::app);
  //       }
  //       else
  //       {
  //         // The file does not exist yet
  //         csv_cum_ofstream.open(full_csv_name);
  //         csv_cum_ofstream << "avg_PRR,avg_latency_ms" << std::endl;
  //       }

  //       csv_cum_ofstream << "," << prrSup->getAveragePRR () << "," << prrSup->getAverageLatency () << std::endl;
  //     }
  //     std::cout << "Average PRR: " << prrSup->getAveragePRR () << std::endl;
  //     std::cout << "Average latency (ms): " << prrSup->getAverageLatency () << std::endl;
  //   }

  return 0;
}

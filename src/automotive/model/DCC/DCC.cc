/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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

* Created by:
    *  Diego Gasco, Politecnico di Torino (diego.gasco@polito.it, diego.gasco99@gmail.com)
*/

#include "DCC.h"

namespace ns3 {
NS_LOG_COMPONENT_DEFINE("DCC");

TypeId
DCC::GetTypeId ()
{
  static TypeId tid = TypeId("ns3::DCC")
                          .SetParent <Object>()
                          .AddConstructor <DCC>();
  return tid;
}

DCC::DCC () = default;

DCC::~DCC()
= default;

std::unordered_map<DCC::ReactiveState, DCC::ReactiveParameters> DCC::getConfiguration(double Ton, double currentCBR)
{
    std::unordered_map<DCC::ReactiveState, DCC::ReactiveParameters> map;
    ReactiveState old_state = m_current_state;
    if (currentCBR >= map[m_current_state].cbr_threshold && m_current_state != ReactiveState::Restrictive)
    {
      m_current_state = static_cast<ReactiveState>(m_current_state + 1);
    }
    else
    {
      if (m_current_state != ReactiveState::Relaxed)
      {
        ReactiveState prev_state = static_cast<ReactiveState> (m_current_state - 1);
        if (currentCBR <= map[prev_state].cbr_threshold)
        {
          m_current_state = static_cast<ReactiveState>(m_current_state - 1);
        }
      }
    }
    if (old_state != m_current_state)
    {
      if (Ton < 0.5)
      {
        map = m_reactive_parameters_Ton_500_us;
      }
      else if (Ton < 1)
      {
        map = m_reactive_parameters_Ton_1ms;
      }
      else
      {
        // Default
        map = m_reactive_parameters_Ton_1ms;
      }
    }
    return map;
}

void DCC::SetupDCC(std::string item_id, Ptr<Node> node, std::string modality, uint32_t dcc_interval, Ptr<MetricSupervisor> traci_client)
{
  m_item_id = item_id;
  m_node = node;
  NS_ASSERT_MSG (modality == "adaptive" || modality == "reactive", "DCC modality can be only "adaptive" or "reactive"");
  m_modality = modality;
  m_dcc_interval = modality == "reactive" ? dcc_interval : dcc_interval / 2;
  m_traci_client = traci_client;
}

void DCC::StartDCC()
{
  if (m_modality == "adaptive")
  {
    if (m_caServiceV1 != nullptr) m_caServiceV1->setAdaptiveDCC();
    if (m_caService != nullptr) m_caService->setAdaptiveDCC();
    if (m_cpServiceV1 != nullptr) m_cpServiceV1->setAdaptiveDCC();
    if (m_cpService != nullptr) m_cpService->setAdaptiveDCC();
    if (m_vruService != nullptr) m_vruService->setAdaptiveDCC();
    adaptiveDCC();
  }
  else
  {
    reactiveDCC();
  }
}

void DCC::reactiveDCC()
{
  NS_LOG_INFO("Starting DCC check");
  NS_ASSERT_MSG (m_metric_supervisor != nullptr, "Metric Supervisor not set");
  NS_ASSERT_MSG (m_dcc_interval != -1, "DCC interval not set");

  double currentCBR = m_metric_supervisor->getCBRPerItem(m_item_id);
  // Get the NetDevice
  Ptr<NetDevice> netDevice = m_node->GetDevice (0);
  Ptr<WifiNetDevice> wifiDevice;
  Ptr<WifiPhy> phy80211p = nullptr;
  // Get the WifiNetDevice
  wifiDevice = DynamicCast<WifiNetDevice> (netDevice);
  if (wifiDevice == nullptr)
  {
    NS_FATAL_ERROR("WiFi Device object not found.");
  }
  // Get the PHY layer
  phy80211p = wifiDevice->GetPhy ();

  if (m_caServiceV1 != nullptr)
  {
    double Ton = m_caServiceV1->getTon(); // Milliseconds
    std::unordered_map<ReactiveState, ReactiveParameters> map = getConfiguration(Ton, currentCBR);
    if (!map.empty())
    {
      phy80211p->SetTxPowerStart (map[m_current_state].tx_power);
      phy80211p->SetTxPowerEnd (map[m_current_state].tx_power);
      phy80211p->SetRxSensitivity (map[m_current_state].sensitivity);
      m_caServiceV1->setNextCAMDCC (map[m_current_state].tx_inter_packet_time);
    }
  }

  if (m_caService != nullptr)
  {
    double Ton = m_caService->getTon(); // Milliseconds
    std::unordered_map<ReactiveState, ReactiveParameters> map = getConfiguration(Ton, currentCBR);
    if (!map.empty())
    {
      phy80211p->SetTxPowerStart (map[m_current_state].tx_power);
      phy80211p->SetTxPowerEnd (map[m_current_state].tx_power);
      phy80211p->SetRxSensitivity (map[m_current_state].sensitivity);
      m_caService->setNextCAMDCC (map[m_current_state].tx_inter_packet_time);
    }
  }

  if (m_cpServiceV1 != nullptr)
  {
    double Ton = m_cpServiceV1->getTon(); // Milliseconds
    std::unordered_map<ReactiveState, ReactiveParameters> map = getConfiguration(Ton, currentCBR);
    if (!map.empty())
    {
      phy80211p->SetTxPowerStart (map[m_current_state].tx_power);
      phy80211p->SetTxPowerEnd (map[m_current_state].tx_power);
      phy80211p->SetRxSensitivity (map[m_current_state].sensitivity);
      m_cpServiceV1->setNextCPMDCC (map[m_current_state].tx_inter_packet_time);
    }
  }

  if (m_cpService != nullptr)
  {
    double Ton = m_cpService->getTon(); // Milliseconds
    std::unordered_map<ReactiveState, ReactiveParameters> map = getConfiguration(Ton, currentCBR);
    if (!map.empty())
    {
      phy80211p->SetTxPowerStart (map[m_current_state].tx_power);
      phy80211p->SetTxPowerEnd (map[m_current_state].tx_power);
      phy80211p->SetRxSensitivity (map[m_current_state].sensitivity);
      m_cpService->setNextCPMDCC (map[m_current_state].tx_inter_packet_time);
    }
  }

  if (m_vruService != nullptr)
  {
    double Ton = m_vruService->getTon(); // Milliseconds
    std::unordered_map<ReactiveState, ReactiveParameters> map = getConfiguration(Ton, currentCBR);
    if (!map.empty())
    {
      phy80211p->SetTxPowerStart (map[m_current_state].tx_power);
      phy80211p->SetTxPowerEnd (map[m_current_state].tx_power);
      phy80211p->SetRxSensitivity (map[m_current_state].sensitivity);
      m_vruService->setNextVAMDCC (map[m_current_state].tx_inter_packet_time);
    }
  }  

  Simulator::Schedule(MilliSeconds(m_dcc_interval), &DCC::reactiveDCC, this);
}

void DCC::adaptiveDCC()
{
  NS_LOG_INFO ("Starting DCC check");
  NS_ASSERT_MSG (m_traci_client != nullptr, "TraCI client not set");
  NS_ASSERT_MSG (m_metric_supervisor != nullptr, "Metric Supervisor not set");
  NS_ASSERT_MSG (m_dcc_interval != -1.0, "DCC interval not set");

  if (!m_time_to_do_steps)
  {
    m_previous_cbr = m_metric_supervisor->getCBRPerItem(m_item_id);
    m_time_to_do_steps = true;
  }
  else
  {
    double currentCBR = m_metric_supervisor->getCBRPerItem(m_item_id);
    double delta_offset;
    // Step 1
    if (m_CBR_its != -1)
      {
        m_CBR_its = 0.5 * m_CBR_its + 0.25 * ((currentCBR + m_previous_cbr) / 2);
      }
    else
      {
        m_CBR_its = (currentCBR + m_previous_cbr) / 2;
      }
    // Step 2
    double factor1 = m_beta * (m_CBR_target - m_CBR_its);
    if ((m_CBR_target - m_CBR_its) > 0)
      {
        delta_offset = factor1 < m_Gmax ? factor1 : m_Gmax;
      }
    else
      {
        delta_offset = factor1 > m_Gmin ? factor1 : m_Gmin;
      }

    // Step 3
    m_delta = (1 - m_alpha) * m_delta + delta_offset;

    // Step 4
    if (m_delta > m_delta_max)
      {
        m_delta = m_delta_max;
      }

    // Step 5
    if (m_delta < m_delta_min)
      {
        m_delta = m_delta_min;
      }

    if (m_caService != nullptr)
      {
        m_caService->toffUpdateAfterDeltaUpdate (m_delta);
      }
    if (m_caServiceV1 != nullptr)
      {
        m_caServiceV1->toffUpdateAfterDeltaUpdate (m_delta);
      }
    if (m_cpService != nullptr)
      {
        m_cpService->toffUpdateAfterDeltaUpdate(m_delta);
      }
    if (m_cpServiceV1 != nullptr)
      {
        m_cpServiceV1->toffUpdateAfterDeltaUpdate(m_delta);
      }
    if (m_vruService != nullptr)
      {
        m_vruService->toffUpdateAfterDeltaUpdate(m_delta);
      }
    m_time_to_do_steps = false;
  }

  Simulator::Schedule(MilliSeconds(m_dcc_interval), &DCC::adaptiveDCC, this);
}

}


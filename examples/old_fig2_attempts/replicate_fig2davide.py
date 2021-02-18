"""
/*
 * This script attempts to replicate Fig 2 of Davide's A Thorough Study of LoRaWAN Performance 
 *  Under Different Parameter Settings paper.
 * 
 * Sim setup
 * No shadowing
 * Trying 100% confirmed and  100% unconfirmed traffic
 * num of trans = 8 (confirmed traffic)
 * 1200 devices (from Martina's email)
 * data rate sweeped from 10^-2 to 2 pkt/s by script
 * radius 6300m (from email with Martina)
 * 1 gateway
 * simulation time = 3*appPeriod (from Martina's email)
 * Stat collection is only done on the second period's time interval (from Martina's email)
 * no ADR, using set SF up
 * packet size a uniform random value between 0 and 40 (from Martina's email)
 * GW DC enabled (this is so by default in gateway-lorawan-mac.cc)
 * TX is priorities (this is so by default m_txPriority in lora-phy-helper.h)
 * Sub-band prioritization is Off (by default, sim needs additional code to enable this)
 * RX2 data rate is SF12 (LorawanMacHelper setup for EU function sets this)
 * Number of TX attempts, m = 8 (set in this script  edLorawanMac->SetMaxNumberOfTransmissions (numberOfTransmissions);)
 * Full duplex = No (by default, sim needs additional code for this)
 * Reception paths = 8 (LorawanMacHelper setup for EU function sets this)
 */
"""

import sem
import numpy as np
import matplotlib.pyplot as plt


# Create our SEM campaign
script = 'replicate-fig2davide'
results_dir = 'newreference-results'

#CountMacPacketsGloballyCpsr is saved under cspr
#CountMacPacketsGlobally in saved under glob

ns_3_dir = '../../../'

campaign = sem.CampaignManager.new(ns_3_dir, script, results_dir,
                                   check_repo=False, overwrite=False)

# Parameter space
#################


#The period in seconds to be used by periodically transmitting applications
appPeriod_values = [] 

#Whether or not end devices require an ACK
confirmed_values = [False, True]

#Number of repeats (runs) to do for averaging
runs = 10

# I don't know the exact x-values used, picking my own.
arrival_rates = list(np.logspace(start= -2, stop=1, num=14))

#Fig 2 only plots till 2 pkt/s, but I generated up till 10 pkt/s.
#arrival_rates = arrival_rates[:-3] #they only plot till 2, dropping last 3 generated rates.

num_devices = 1200
for arrival_rate in arrival_rates:

    # 1200 devices will send in 1 appPeriod 1200 packets
    # different appPeriods will generated different arrival rates (lamdba)
    # appPeriod = 1200/(lambda)

    appPeriod_values.append(np.ceil(num_devices/arrival_rate))
    print("For ", np.round(arrival_rate,3), " pkt/s, arrival rate for 1200 devices must be ",  appPeriod_values[-1], "s")





param_combinations = {
    'appPeriod': appPeriod_values,
    'confirmed' : confirmed_values,
}
print(param_combinations)

campaign.run_missing_simulations(param_combinations, runs)

confirmed_flag = False

def get_phy_receivedPackets(result):
    """
    Extract the probability of success from the simulation output.
    From lora-packet-tracker.cc
    Vector packetCounts will contain the following fields: totPacketsSent receivedPackets
    interferedPackets noMoreGwPackets underSensitivityPackets noMoreTx
    """

    outcomes = [float(a) for a in result['output']['stdout'].split()]
    if(confirmed_flag == False):
        return outcomes[1]/outcomes[0] #receivedPackets/totPacketsSent

    if(confirmed_flag == True):
        return outcomes[1]/1200 #outcomes[0] #interferedPackets/totPacketsSent


#Plot unconfirmed 
param_combinations = {
    'appPeriod': appPeriod_values,
    'confirmed' : [False],
}

results_unconfirmed = campaign.get_results_as_xarray(param_combinations,
                                            get_phy_receivedPackets, '', runs)

print("\nResults for get_phy_receivedPackets \n")
results_average = results_unconfirmed.reduce(np.mean, 'runs')
results_std = results_unconfirmed.reduce(np.std, 'runs')

avg = results_average.sel(confirmed = False)
std = results_std.sel(confirmed = False)

print("ULPDR Values for unconfirmed traffic")
ULPDR = np.squeeze(avg).values
for index,item in enumerate(arrival_rates):
    print(np.round(item,3), " pkt/s: ",np.round(ULPDR[index],3))    
      

plt.plot(arrival_rates, np.squeeze(avg),  'ok-' )

#Plot confirmed
confirmed_flag = True

param_combinations = {
    'appPeriod': appPeriod_values,
    'confirmed' : [True],
}

results_confirmed = campaign.get_results_as_xarray(param_combinations,
                                            get_phy_receivedPackets, '', runs)

results_average = results_confirmed.reduce(np.mean, 'runs')
results_std = results_confirmed.reduce(np.std, 'runs')

avg = results_average.sel(confirmed = True)
std = results_std.sel(confirmed = True)
plt.plot(arrival_rates, np.squeeze(avg),'xk-' )

print("ULPDR Values for confirmed traffic")
ULPDR = np.squeeze(avg).values
for index,item in enumerate(arrival_rates):
    print(np.round(item,3), " pkt/s: ",np.round(ULPDR[index],3))    
      
plt.xscale('log')
plt.grid(True, 'both', axis = 'both') #log graph

plt.ylim([0.4,1.01])
plt.xlim([0.01,10.1])
plt.yticks([0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1])

plt.legend(['Unconfirmed', 'Confirmed'], title = "Traffic type", loc=(1.01,0.5))
plt.subplots_adjust(right=0.75) #add space to the right so that legend is neatly visible

plt.xlabel("$\lambda$ [pkt/s]")
plt.ylabel("UL-PDR")
plt.title("")

plt.show()

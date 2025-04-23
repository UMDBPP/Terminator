# Board Heater

Ok so the little thermo-couple thermostat thing I chose works great. Turns on and off the NMOS FET perfectly and the long trace embeded in the board is very clearly able to heat it. The data in the 04/21/2025 graph shows that the heater brought the temp up from -15 C twice and then seemed to give up? That or it looks like it was just continuously on since the cooling happened much slower.

It only ran for ~43 minutes from what I can tell and restarted a lot once it reached about -28 C. The battery is also not very discharged so I'm not surprised that it didn't run very long. I'm going to re-run this test and log board temperature as well. My only other thought is that the battery got too cold for the amount of current the board heater draws so the system started dropping out. Not sure the best way to go about attaching the battery to the board since the back needs to be open for flight line to pass over.

Additionally since the system runs at 3.3V, maybe a 2S LiPo would be a better fit. Options are:
1. https://hobbyking.com/en_us/turnigy-nano-tech-300mah-2s-35c-lipo-pack-w-jst-ph-connector.html
2. https://hobbyking.com/en_us/turnigy-nano-tech-450mah-2s-65c-lipo-e-flite-compatible-blade-130x-eflb3002s35.html

# MPSP
micropython science payload for UAS


# LED Status
On boot up Green and Orange are on for 2 seconds
If hold down UserSwitch at this point MPSP will boot into STANDBY Mode. This allows User to connect to microSD card
from desktop/laptop

- Green Flashing at 1Hz == Status Good, MPSP main loop is running and there is a heartbeat from the flight computer
- Red Flashing at 10hz == Main Loop is running but no heartbeat in last 5 seconds.

@echo off
REM Open SSH sessions to all rover Jetsons in Windows Terminal tabs
REM RV1: 2 tabs (debug), RV2: 1 tab, Simulator: 1 tab
REM Uses SSH key auth — run setup first (see tools/README or CLAUDE.md)

set KEY=%USERPROFILE%\.ssh\agri_rover

start wt ^
  new-tab --title "RV1-A" ssh -t -i %KEY% ilasa1@192.168.100.19 "cd ~/Desktop/ros_agri_rover && bash" ; ^
  new-tab --title "RV1-B" ssh -t -i %KEY% ilasa1@192.168.100.19 "cd ~/Desktop/ros_agri_rover && bash" ; ^
  new-tab --title "RV2-A" ssh -t -i %KEY% ilasa1@192.168.100.20 "cd ~/Desktop/ros_agri_rover && bash" ; ^
  new-tab --title "RV2-B" ssh -t -i %KEY% ilasa1@192.168.100.20 "cd ~/Desktop/ros_agri_rover && bash" ; ^
  new-tab --title "SIM"   ssh -t -i %KEY% ilasa1@192.168.100.22 "cd ~/Desktop/ros_agri_rover && bash"

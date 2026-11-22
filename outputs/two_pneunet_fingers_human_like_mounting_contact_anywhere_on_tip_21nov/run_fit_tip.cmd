@echo off
set PYTHONIOENCODING=utf-8
set OMP_NUM_THREADS=1
cd /d C:\Users\geo\Documents\Projects\gramian-map-workshop
C:\Users\geo\Documents\Projects\ARIA\Wave_manip\inhand\Scripts\python.exe -u tools\two_pneunet_fingers_human_like_mounting_contact_anywhere_on_tip_reach_check_21nov.py --wave-manip C:/Users/geo/Documents/Projects/ARIA/Wave_manip --gramian-map C:/Users/geo/Documents/Projects/ARIA/Gramain_extraction/gramian_map --workers 4 --contact tip > outputs/two_pneunet_fingers_human_like_mounting_contact_anywhere_on_tip_21nov\fit_log_tip.txt 2>&1

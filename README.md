# ASL-version of the Pixhawk/PX4 Firmware #

##Guidelines##

###General###
 - Keep code (structure, formatting, variables) as close to PX4 as possible. Be ready to push our stuff back to PX4 using Pull Requests.

###Branches###
 - stable: Current stable release, flight tested
 - master: Current main release, i.e. under development but bench-tested and thus operational.
 - features/MYFEATURENAME: A new feature branch. All new features shall be added like this.
 - fix/MYFIX: A new fix. All new fixes shall be added like this.

###Commits###
 - Commit Message: Please use the "Module: Message" format, where Module is the approximate location or scope of your change (e.g. Mavlink, aslctrl, sensors) and Message is your ordinary message.
 
###Other:###
 - TAGGING: Use tags (done via "git tag") to name specific releases and milestones, e.g. those used for a specific flight test ("AS-1 TF#7", "Techpod TF#XX").
 - ISSUES: Use the github issue tracking & bug reporting system
 
###Resources###
 - Pixhawk Wiki: https://pixhawk.org
 - Pixhawk Developer Tour : https://pixhawk.org/dev/start
 - Pixhawk Forum: http://groups.google.com/group/px4users

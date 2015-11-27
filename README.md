# ASL-version of the Pixhawk/PX4 Firmware #

##Guidelines##

This repository follows standard PX4 and ASL coding guidelines, most notably a strict _pull-request-to-master_ workflow and a _master/stable/feature/fix/..._ branch structure.

###General###
 - Keep code (structure, formatting, variables) as close to PX4 as possible. Be ready to push our stuff back to PX4 using Pull Requests.

###Branches###
 - stable: Current stable release, flight tested
 - master: Current main release, i.e. under development but bench-tested and thus operational. Please do not push directly to master, but use Pull Requests (PR) to contribute to master (see below!).
 - features/MYFEATURENAME: A new feature branch. All new features shall be added like this.
 - fix/MYFIX: A new fix. All new fixes shall be added like this.

###Commits###
 - Commit Target: Direct commits to master are _NOT allowed_ ! Please create a new branch (fix/feature/...), push to github, and create a pull request w.r.t. master. This follows the standard code review process at ASL (see https://github.com/ethz-asl/programming_guidelines/wiki/Code-review-process)
 - Commit Message: Please use the "Module: Message" format, where Module is the approximate location or scope of your change (e.g. Mavlink, aslctrl, sensors) and Message is your ordinary message.
 
###Other:###
 - TAGGING: Use tags (done via "git tag") to name specific releases and milestones, e.g. those used for a specific flight test ("AS-1 TF#7", "Techpod TF#XX").
 - ISSUES: Use the github issue tracking & bug reporting system
 
###Installation:###
 - `git clone` this repository (e.g. into ../PX4/Firmware)
 - Do the usual `submodule init` and `submodule update` to update your submodules
 
###Resources###
 - Pixhawk Wiki: https://pixhawk.org
 - Pixhawk Developer Tour : https://pixhawk.org/dev/start
 - Pixhawk Forum: http://groups.google.com/group/px4users
 - ASL Code Review process: https://github.com/ethz-asl/programming_guidelines/wiki/Code-review-process

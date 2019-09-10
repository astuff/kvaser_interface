^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kvaser_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.1.0 (2019-09-10)
------------------
* Merge pull request `#45 <https://github.com/astuff/kvaser_interface/issues/45>`_ from astuff/fix/slow_response
  Fix/slow response
* Read optimization when dlc != 0.
* Removing unecessary include.
* Simplifying isOpen() for speed.
* Merge pull request `#42 <https://github.com/astuff/kvaser_interface/issues/42>`_ from astuff/maint/launch_default_screen_output
* Default launch file to output to screen.
* Merge pull request `#40 <https://github.com/astuff/kvaser_interface/issues/40>`_ from astuff/maint/node_tests
  Node-Level Testing
* Removing unused reader in test.
* CI: Forgot to update docker call with extra devices on Melodic.
* Merge pull request `#39 <https://github.com/astuff/kvaser_interface/issues/39>`_ from astuff/fix/publishing_dlc_0
  Empty (valid) messages not being published.
* Merge pull request `#38 <https://github.com/astuff/kvaser_interface/issues/38>`_ from astuff/fix/dlc_mismatch
* Was not publishing correct, empty messages. Fixed.
* Msgs with dlc < 8 would fail. Fixed.
* Contributors: Joshua Whitley, Sam Rustan

4.0.3 (2019-04-26)
------------------
* Fixing additional bug with setting flags.
* Added flag tests.
* Adding comparison operators to custom structs.
* Adding more unit tests.
* Merge pull request `#31 <https://github.com/astuff/kvaser_interface/issues/31>`_ from astuff/fix/extended_id_transmission
* Hotfix: Flags were not being propery set on transmitted messages.
* Adding basic tests and infrastructure for node tests.
* Merge pull request `#32 <https://github.com/astuff/kvaser_interface/issues/32>`_ from astuff/maint/ci_remove_lunar
* CI: Remove Lunar (EOL) build.
* Contributors: Daniel-Stanek, Joshua Whitley

4.0.2 (2019-03-31)
------------------
* Merge pull request `#29 <https://github.com/astuff/kvaser_interface/issues/29>`_ from astuff/hotfix/reader_bug
* No Longer Closes Channel After Read
* Contributors: Joshua Whitley

4.0.1 (2019-03-31)
------------------
* Merge pull request `#28 <https://github.com/astuff/kvaser_interface/issues/28>`_ from astuff/hotfix/can_reader_callback
* Fixing bug in ROS callback.
* Contributors: Joshua Whitley

4.0.0 (2019-03-27)
------------------
* After merging of PR #21, ros_linuxcan is now ros_kvaser_interface and has several API changes.
* Merge pull request `#21 <https://github.com/astuff/kvaser_interface/issues/21>`_ from astuff/feat/callbacks
* Changing canmonitor to polling loop at 100Hz.
* Adding comment about CAN FD DLC/payload size.
* Normalizing shutdowns in canmonitor.
* Renaming file and library for consistency.
* Normalizing error messages between open() functions.
* Having read proxy hold copy of CanHandle for lifetime management.
* Created read callback register func and proxy.
* write() now uses CanMsg.
* read() now returns all message types. Up to user to filter.
* Changing read() to use CanMsg. Added util funcs.
* Added CanMsg message struct.
* Contributors: Joshua Whitley

3.1.0 (2019-03-19)
------------------
* Merge pull request `#20 <https://github.com/astuff/kvaser_interface/issues/20>`_ from astuff/feat/more_tools
  Adding canmonitor tool.
* Updating README with new tools.
* Adding basic list_channels.
* Adding KvaserCanUtils and ability to pull channels.
* Merge pull request `#19 <https://github.com/astuff/kvaser_interface/issues/19>`_ from astuff/maint/add_lint_to_run_tests
  Maint/add lint to run tests
* Merge pull request `#18 <https://github.com/astuff/kvaser_interface/issues/18>`_ from astuff/maint/scoped_enum
  Changing ReturnStatuses to Scoped Enum
* Merge pull request `#17 <https://github.com/astuff/kvaser_interface/issues/17>`_ from astuff/maint/lint_cleanup
  Removing some rule exceptions from roslint.
* Contributors: Joshua Whitley

3.0.0 (2019-01-23)
------------------
* Merge pull request `#14 <https://github.com/astuff/kvaser_interface/issues/14>`_ from astuff/memory-management
* Adding roslint and formatting clean up.
  Testing better memory management with smart pointers.
  Contains significant API changes (function names and include header location)
  to conform to ROS C++ guidelines.
* Merge pull request `#13 <https://github.com/astuff/kvaser_interface/issues/13>`_ from astuff/fix/short_messages
* Fixes issues seen when receiving short messages
  Before: Short messages (<8 bytes) would correctly report
  DLC and any bytes sent, but published topic would contain
  garbage extra data in unused bytes.
  After: Short messages are published with zeros for any unused
  bytes instead of line noise.
* Merge pull request `#11 <https://github.com/astuff/kvaser_interface/issues/11>`_ from giuspen/is_extended
* set value of can_pub_msg.is_extended from read
* Contributors: Daniel-Stanek, Giuseppe Penone, Joe Driscoll, Joshua Whitley, Sam Rustan, Zach Oakes

2.0.2 (2018-09-13)
------------------
* Bumping version.
* Updating README with PPA and other information.
* Cleaning up install script.
* Adding melodic build. Fixing allowed_failures.
* Fixing license in package.xml.
* Fixed bug causing extremely high CPU utilization.
* Fixing intermittent pause while checking can bus status.
* corrects hard coding of sending extended IDs to use is_extended member
* Updating package.xml to format 2.
* Re-releasing under MIT license.
* Adding install rule for launch file.
* Removing debugging messages.
* Setting Kvaser to not close on each write loop.
* Changing writer to regular spin instead of async spinner.
* Reducing number of threads used for can_write.
* Reducing read time pause.
* Turning off can_echo.
* Changing script name to be more clear.
* Cutting down on unecessary error messages.
* Updating example launch file to match name changes.
* Final changes for name change.
* Preparing for name change to kvaser_interface.
* Adding bit_rate and example launch file.
* Adding node.
* Moving Travis CI build status image in README.
* Bumping version and cleaning up package.xml.
* Changing many function params to const ref.
* Modifying CPATH for linuxcan install.
* Missed a state in is_open.
* Adding the is_open function.
* Add optional flag to open routine to turn off tx echo
* Fixing license typos.
* Changing BAD_PARAMS to BAD_PARAM.
* Moved return_status_desc to utils.cpp for general use.
* Changed CHANNEL_NOT_OPEN to CHANNEL_CLOSED.
* Adding CHANNEL_NOT_OPEN error.
* Added CLOSE_FAILED. Made errors negative numbers. Added NO_CHANNELS_FOUND.
  Making the return statuses more standardized between can_interface and network_interface.
  Added the new return_statuses to the return_statuses_desc function.
* Adding return_status_desc function.
* Changing license to GPLv3.
* Removing CanFrame in favor of can_msgs/Frame.
* Changing size of id field to handle extended IDs (whoops).
* Changing can_frame to CanFrame.
* Fix loop in read routine to skip over TX ACK and other protocol type messages that the higher level application doesn't need.
* Bypassed guts of open function if handle is already on-bus.
* Making devel version match install version.
* Adding basic README.
* Moved canBusOn.
* Going on bus in read/write instead of open.
* Close channel in destructor if still valid.
* Changing names to conform to ROS C++ style guide.
* Adding repository URL to package.
* Initial commit.
* Contributors: Christopher Vigna, Daniel Stanek, Joe Kale, Joshua Whitley, Lucas Buckland, Sam Rustan, driscoll85

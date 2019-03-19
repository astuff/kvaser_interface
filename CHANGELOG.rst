^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kvaser_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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

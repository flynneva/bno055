^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bno055
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2024-02-17)
------------------
* Bump to 0.5.0 to prep for release
* Added Gravity publisher (`#64 <https://github.com/flynneva/bno055/issues/64>`_)
  * Added Gravity publisher
  * Updated Readme to introduce Gravity publisher
* Spelling/Grammar (`#62 <https://github.com/flynneva/bno055/issues/62>`_)
* Fix uart write answer reading (`#57 <https://github.com/flynneva/bno055/issues/57>`_)
* Update package.xml to include smbus dependency (`#58 <https://github.com/flynneva/bno055/issues/58>`_)
* fix build error on foxy that caused by typo (`#59 <https://github.com/flynneva/bno055/issues/59>`_)
* Contributors: Andrew Symington, Burak Guler, Combinatrix, Evan Flynn, Vintheruler1, emilnovak

0.4.1 (2023-01-15)
------------------
* Update CHANGELOG
* Bump patch version after small bug fix
* Fix i2c issues found (param and division by zero) (`#56 <https://github.com/flynneva/bno055/issues/56>`_)

0.4.0 (2023-01-13)
------------------
* Bump version for new release
* Update CHANGELOG for new release
* Improve the doc strings for the connectors
* Add I2C implementation (`#52 <https://github.com/flynneva/bno055/issues/52>`_)
* Merge pull request `#50 <https://github.com/flynneva/bno055/issues/50>`_ from flynneva/fix-invalid-log-print
  Fix invalid log string variable expansion
* Update CI to include Humble
* Fix invalid log string variable expansion
* Merge pull request `#49 <https://github.com/flynneva/bno055/issues/49>`_ from sjev/operation_mode
* add device mode setting
* Contributors: Andrew Symington, Evan Flynn, Jev 

0.3.0 (2022-03-22)
------------------
* Merge pull request `#43 <https://github.com/flynneva/bno055/issues/43>`_ from flynneva/develop
  bring over updates
* Merge pull request `#44 <https://github.com/flynneva/bno055/issues/44>`_ from Towflos/develop
* fixed setting of calibration data, added default values for mag and acc radius
* added example_interfaces to package.xml, added service information to README.md
* cleanup
* prefix added to service
* removed seq, fixed parameters, refactored reading of calibratio data, added service to request calibration data
* added seq and time to message headers
* mag_radius and accel_radius added to get_calib_offsets()
* mag radius and acc radius added as parameter
* fix greetings for incoming PRs from forks
  see [this nice write up](https://github.com/SalesforceLabs/LightningWebChartJS/pull/95) for details
* Merge pull request `#42 <https://github.com/flynneva/bno055/issues/42>`_ from deepinbubblegum/launch_develop
  Launch develop
* edit readme launch file
* add launch file
* Merge pull request `#41 <https://github.com/flynneva/bno055/issues/41>`_ from flynneva/develop
  bring over 0.2.0 updates to main
* Merge pull request `#32 <https://github.com/flynneva/bno055/issues/32>`_ from flynneva/develop
  initial docs up and running
* Merge pull request `#31 <https://github.com/flynneva/bno055/issues/31>`_ from flynneva/develop
  source galactic not rolling
* Merge pull request `#30 <https://github.com/flynneva/bno055/issues/30>`_ from flynneva/develop
  dont use forked repo for setup-ros
* Merge pull request `#29 <https://github.com/flynneva/bno055/issues/29>`_ from flynneva/develop
  fix setup-ros version to 0.2
* Contributors: Evan Flynn, Florian Herrmann, deepinbubblegum

0.2.0 (2021-12-6)
-----------------
* Merge pull request `#36 <https://github.com/flynneva/bno055/issues/36>`_ from flynneva/feature/add-covariance
  [33] add default covariance values, make them configurable
* fix printout for parameters
* [33] add defaults for magnetic field covariance values
* [37] add logic to set offsets
* [33] add default covariance values, make them configurable
* Merge pull request `#24 <https://github.com/flynneva/bno055/issues/24>`_ from flynneva/fix/scaling_factors
* [35] add back in comm constants, modify variable names as needed to fix bug introduced by `#16 <https://github.com/flynneva/bno055/issues/16>`_
* [34] use underscores in setup.cfg instead of dashes
* [23] fix acc and mag scaling factors and make them configurable
* only run docs ci on main updates
* use sh instead of bash script for docs ci
* use . instead of source for docs ci
* remove -r from pip install
* use relative paths for docs ci
* use absolute paths for docs ci
* ls in docs ci
* switch doc generation ci to pre-built docker image
* source galactic not rolling
* Merge branch 'develop' of github.com:flynneva/bno055 into develop
* dont use forked repo for setup-ros
* Merge branch 'main' into develop
* fix setup-ros version to 0.2
* Merge pull request `#28 <https://github.com/flynneva/bno055/issues/28>`_ from flynneva/develop
  fix setup ros version to v0.2
* fix setup ros version to v0.2
* Merge pull request `#27 <https://github.com/flynneva/bno055/issues/27>`_ from flynneva/develop
  use galactic for doc generation
* Merge pull request `#26 <https://github.com/flynneva/bno055/issues/26>`_ from flynneva/feature/sphinx_docs
  use galactic for doc generation, not rolling
* use galactic for doc generation, not rolling
* Merge pull request `#25 <https://github.com/flynneva/bno055/issues/25>`_ from flynneva/develop
  bring over updates to main, generate docs for first time
* Merge pull request `#16 <https://github.com/flynneva/bno055/issues/16>`_ from flynneva/feature/sphinx_docs
  Feature/sphinx docs
* fix docs ci path when uploading docs to gh-pages
* Merge branch 'develop' into feature/sphinx_docs
* add modules to docs and update registers
* Merge pull request `#21 <https://github.com/flynneva/bno055/issues/21>`_ from flynneva/develop
  normalize quaternion
* Merge pull request `#20 <https://github.com/flynneva/bno055/issues/20>`_ from flynneva/fix/normalize_quaternion
  normalize quaterion
* normalize quat
* Merge pull request `#19 <https://github.com/flynneva/bno055/issues/19>`_ from flynneva/feature/prebuilt_docker_ci
  move to pre-built ros docker images
* move to pre-built ros docker images
* minor docs updates
* doc page templates
* starting on docs
* Merge pull request `#13 <https://github.com/flynneva/bno055/issues/13>`_ from flynneva/develop
  bump for release
* Merge pull request `#10 <https://github.com/flynneva/bno055/issues/10>`_ from flynneva/develop
  bring over updates to main
* Contributors: Evan Flynn, flynneva

0.1.1 (2021-02-04)
------------------
* add changelog
* Contributors: flynneva

0.0.1 (2021-02-04)
------------------
* Merge pull request `#12 <https://github.com/flynneva/bno055/issues/12>`_ from flynneva/feature/release_ci
  add release actions
* add release actions
* Merge pull request `#11 <https://github.com/flynneva/bno055/issues/11>`_ from flynneva/fix/README_updates
  update readme
* update readme
* Merge pull request `#9 <https://github.com/flynneva/bno055/issues/9>`_ from flynneva/fix/bus_overrun_error
  Fix/bus overrun error
* fix flake8 errors & recommended changes
* added bus_over_run_error to exceptions
* add pycache to .gitignore
* Merge pull request `#8 <https://github.com/flynneva/bno055/issues/8>`_ from flynneva/fix/remove_eol_distro
  remove eloquent from ci
* Merge pull request `#7 <https://github.com/flynneva/bno055/issues/7>`_ from flynneva/fix/no_timer_shutdown
  only shutdown timers if timers are available
  merging this since it is a minor change
* remove eloquent from ci
* seperate log output into two lines for readability
* only shutdown timers if timers are available
* Merge pull request `#6 <https://github.com/flynneva/bno055/issues/6>`_ from flynneva/fix/flake8
  Fix/flake8
* add flake8 ignore mechanism to README
* ignore B902 blind except flake8 error
* copyright fixes
* fixed flake8 & pep257 errors
* starting to fix flake8 errors
* Merge pull request `#4 <https://github.com/flynneva/bno055/issues/4>`_ from flynneva/feature/extend-msgs
  Sensor Placement, Calibration Status, Error Handlng
* - graceful ROS node shutdown on Ctrl+C
* - fix: adjusted calls to transmit()
* - sensor placement is now configurable (axis remap)
* - simplified error handling - now exception-based instead of return values
  - periodic publishing of calibration status (period is configurable)
  - configurable ROS topics (configurable prefix) so that they can be unique
* Merge pull request `#3 <https://github.com/flynneva/bno055/issues/3>`_ from whatis777/develop
  Basic Project refactorings
* Merge pull request `#1 <https://github.com/flynneva/bno055/issues/1>`_ from whatis777/feature/i2c
  Feature/i2c
* - improved UART message error handling & logging
  - introduced UART timeout parameter
  - adjusted default values for improved communication quality
  - renamed UART related parameters
  - added locks to prevent overlapping data queries
* specify ros distro to linter action
* results of code review: fixed flake8 findings
* results of code review
* added UART read & write calls, clean up
* extracted sensor API (use cases) into separate class; main is now also class based
* refactored method names & moved connectors
* extracted ROS node parameter handling
* restructured sources, added documentation, fixed runtime issues with ROS2
* Merge pull request `#2 <https://github.com/flynneva/bno055/issues/2>`_ from flynneva/actions
  add linter and rename actions file
* limit linter to only flake8 for now
* change linter array format
* linter array moved out of matrix
* Merge branch 'actions' of github.com:flynneva/bno055 into actions
* added linter step to actions
* Merge pull request `#1 <https://github.com/flynneva/bno055/issues/1>`_ from flynneva/actions
  added gh actions
* maybe no requirements.txt file needed?
* removed ros2 pkgs from requirements file
* added requirements.txt file
* develop not devel
* added gh actions
* fixed ament_python warning by adding data_files
* extracting UART-specific code, W.I.P.
* added calibration parameters
* updated readme
* added parameters
* Merge branch 'master' into develop
* trying to get_parameter
* added other topics
* Update README.md
* added check if buf is empty
* added back copyright agreement
* first commit
* Contributors: Evan Flynn, Manfred Novotny, flynneva, whatis777

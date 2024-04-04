# tekkneeca
Assistive Robot for Total Knee Arthroplasty

## Installation

- Clone the package

        cd ~/<ros2_ws>/src
        git clone git@github.com:team-paradocs/tekkneeca.git

- Install submodules.

        cd ~/<ros2_ws>/src/tekkneeca
        git submodule init
        git submodule update

- (Optional) fetch latest commit for submodule

        git submodule update --remote --merge

- Build the packages

        cd ~/<ros2_ws>
        colcon build

- source environment

        source ~/<ros2_ws>/install/setup.bash


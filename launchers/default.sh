#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
roscore &
sleep 5
dt-exec rosrun image_getter publisher.py
dt-exec rosrun image_getter subscriber.py
# dt-exec rosrun image_processing decoder_node.py
# dt-exec rosrun image_processing rectifier_node.py

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join

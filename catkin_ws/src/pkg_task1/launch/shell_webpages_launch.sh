#!/bin/bash

# Store URL in a variable
URL0="https://docs.google.com/spreadsheets/d/1ZSl8PBmCYAtqBniifbNA9lWEB33a7_FkKpCYaUT-o_U/edit#gid=0"
URL1="http://www.hivemq.com/demos/websocket-client/"

# Print message
echo "** Opening $URL0 in Firefox **"
echo "** Opening $URL1 in Firefox **"

# Use firefox to open the URLs in new windows
firefox -new-window $URL0 -new-window $URL1

# Download and unzip the best jams around
curl --output jams.zip --netrc-file vectorsecrets.txt https://files.dylanzeml.in/protected/jams.zip
mkdir -p $HOME/autonav_software_2023/deps/songs
unzip -o jams.zip -d $HOME/autonav_software_2023/deps/songs
rm jams.zip
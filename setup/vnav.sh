# Download and unzip vectornav files
wget https://files.dylanzeml.in/vectornav.zip
sudo mkdir /usr/local/vectornav
sudo chown -R "$USER":"$USER" /usr/local/vectornav
unzip -o vectornav.zip -d /usr/local/vectornav
rm vectornav.zip

# Install vectornav
cd /usr/local/vectornav/python
sudo python3 setup.py install
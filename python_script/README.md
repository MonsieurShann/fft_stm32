# Python script

Here is a quick explanation of why and how this python script exists.

### Why

Once the uC has processed the fft on a signal, we want to plot the results, to visualize the decomposition in frequencies. <br>
As such, the python script is responsible for receiving serial data, and plotting them on a graph. <br>

### How

Make sure to exexute all these command line before running uC code.

\`\`\`sh
# Create python virtual env
python -m venv venv

# Activate virtual env
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Run the soft
python fft.py
\`\`\`
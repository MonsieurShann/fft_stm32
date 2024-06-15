# Python script

Here is a quick explanation of why and how this python script exists.

### Why

Once the uC has processed the fft on a signal, we want to plot the results, to visualize the decomposition in frequencies. <br>
As such, the python script is responsible for receiving serial data, and plotting them on a graph. <br>

### How

Make sure to exexute all these command line before running uC code.

Create Virtual env<br>
```
python -m venv venv
```

Activate virtual env<br>
```
source venv/bin/activate
```

Install dependencies<br>
```
pip install -r requirements.txt
```

Run the soft<br>
```
python fft.py
```
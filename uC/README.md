# uC Software

Hers you'll find attached: <br>
- main.c of uC code (the only one that has been touched) <br>
- fft_test.ioc which contains a lot of hardware and software configuration used: <br>

Important functions are: <br>

The function which takes as argument frequencies of sinuses, and generate associated points. <br>
1. 
```
combine_signals_evolve();
```

The function which takes the signal and perform an fft on it. <br>
2. 
```
perform_fft();
```

The function which sens results to the personal computer. <br>
3. 
```
send_result();
```
import numpy as np

# Convert time series data to frequency domain using FFT
def time_to_frequency_domain(data):
    # Perform FFT on the data
    freq_data = np.fft.fft(data)
    
    # Get the frequency bins
    freq_bins = np.fft.fftfreq(len(data))
    
    return freq_data, freq_bins


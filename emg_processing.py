from biquad import Biquad
from emg import EMG


class EmgProcessing(object):

    def __init__(self, emg):
        """
        =INPUT=
            emg - int
                The number of the emg to be processed (choose between 1 to 3)
        """
        # Set EMG signals
        self.emg = EMG(emg=emg)

        # High pass filter emg with a cutoff frequency of 0.2 Hz
        self.filter_emg_hp_emg = Biquad(
            (1, -1.9982228302789635, 0.9982244080135089), (0.9991118095731181, -1.9982236191462361, 0.9991118095731181))

        # Low pass filter emg with a cutoff frequency of 30 Hz
        self.filter_emg_lp_emg = Biquad(
            (1, -1.9733440008737442, 0.9736946230245347), (0.00008765553769759188, 0.00017531107539518376, 0.00008765553769759188))

        return


    def process_signal(self):
        """
        Returns the signal processed
        """
        # Read emg values
        value_emg = self.emg.read()

        # Apply high pass filter
        data_filtered_emg = self.filter_emg_hp_emg.step(value_emg)

        # Apply rectification
        data_filtered_emg = abs(data_filtered_emg)

        # Apply low pass filter
        emg_processed_value = self.filter_emg_lp_emg.step(data_filtered_emg)

        return emg_processed_value
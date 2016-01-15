import sys, os, logging

sys.path.append(os.path.join('..', 'Common'))

import FuzzyController

if __name__ == "__main__":
    cfile = open(sys.argv[1], 'r')
    config_lines = cfile.readlines()
    cfile.close()

    rootlogger = logging.getLogger()
    rootlogger.setLevel(20)
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(20)
    rootlogger.addHandler(console_handler)

    controller = FuzzyController.FuzzyController()
    controller.initialize(config_lines)

    inputs = list()
    if len(sys.argv) > 2:
        strinputs = sys.argv[2:]
        inputs = [float(a) for a in strinputs]
    print ("output = %g"%(controller.Compute(inputs)))

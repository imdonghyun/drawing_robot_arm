import argparse

def config_parser():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--path', required=True, 
                        help="image path")
    parser.add_argument("--scale", type=int, default=100, 
                        help="image resize scale")
    parser.add_argument("--cannyThreshold", type=list, default=[500, 500], 
                        help="Threshold of Canny. 2 elements list")
    parser.add_argument("--noiseDel", type=bool, default=True, 
                        help="Whether operate noise_del function")
    parser.add_argument("--noiseLen", type=int, default=10,
                        help="threshold of lenght of nosie")
    

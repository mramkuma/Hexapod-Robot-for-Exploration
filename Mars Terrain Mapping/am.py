import csv

import matplotlib.pyplot as plt
import numpy as np
import matplotlib.cm as cm



occ1 = []

# opening the CSV file
with open('BW.txt', mode ='r')as file:
   
  # reading the CSV file
  csvFile = csv.reader(file)
 
  # displaying the contents of the CSV file
  for lines in csvFile:
    occ1.append(lines)


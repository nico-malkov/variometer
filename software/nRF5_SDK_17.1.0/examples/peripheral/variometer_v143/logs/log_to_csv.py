
# Online Python - IDE, Editor, Compiler, Interpreter

import csv
import re

with open('1.log') as logfile:
    lines = logfile.read().splitlines() # Split file to a list called 'lines' (every line is an item in the list)
    lenght = len(lines) # Number of items in the 'lines' list
    lines_new = [] # Empty list for formatted 'lines' list
    for string in lines: # For every line 9which is string) in the 'lines' list
      string = string[3:] # Delete first three characters (the number of port from the log file looking 00>)
      string = re.findall(r"[-+]?(?:\d*\.*\d+)", string) # Extract float-looking value
      string = ';'.join(string) # Join every float-looking number to the according string
      lines_new.append(string) # Add newly formatted string to the line of 'lines_new' list)
    print(lines_new)
    del (lines_new[lenght-1], lines_new[lenght-2]) # Delete last two lines from the list (empty string and datum there)
    del (lines_new[0], lines_new[1]) # Delete first three lines from the list (datum there)
    del (lines_new[0]) # Continuation of the previuos line
    print(lines_new)
    lines_float = [float(x) for x in lines_new] # Make sure items in the list are floats, not strings
    lines_new = [lines_new[x:x+4] for x in range(0, len(lines_new), 4)] # Group lines by 4 in a row
       
    with open('1.csv', 'wb') as csvfile:
        content = csv.writer(csvfile, dialect='excel', delimiter=';')
        content.writerows(lines_new)

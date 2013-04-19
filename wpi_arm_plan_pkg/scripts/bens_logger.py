#!/usr/bin/env python
# Bener Suay 2013 January
# benersuay@wpi.edu

# This script (and function) saves data for the TePRA paper.
# The data belongs success rate of the CBiRRT Planner.

import time
import os
import sys
from datetime import datetime

class BensLogger:
    def __init__(self,arg_note='',arg_name='new'):
        if(arg_note != ''):
            arg_note = '_'+arg_note

        if(arg_name != 'new'):
            self.fname = arg_name+arg_note+'.csv'
        else:
            timestamp = str(datetime.now())
            self.fname = timestamp[0:10]+'_'+timestamp[11:19]+arg_note+'.csv'
            
        self.file = open(self.fname,'w') # for only writing (an existing file with the same name will be erased)
        self.file.close()

    def save(self,entry): # entry should be a list
        self.file = open(self.fname,'a') # opens the file for appending; any data written to the file is automatically added to the end
        for e in range(len(entry)):
            self.file.write(str(entry[e]))
            if(e == len(entry)-1):
                self.file.write('\n')
            else:
                self.file.write(',')
        self.file.close()

    def header(self,labels): # this function is actually redundant but the name makes the purpose easy to understand
        # In the future, even if the file has a lot of data in it, this function should just change / replace the first line.
        self.save(labels)

    def read(self,entry_num):
        # This method is supposed to return the entry_numth entry in the file, to be implemented later.
        pass

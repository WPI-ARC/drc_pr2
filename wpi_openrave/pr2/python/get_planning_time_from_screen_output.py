import os
import sys

def get_total_planning_time(fname):
    f=open(fname,'r')
    output=f.read()
    f.close()
    soutput = output.split()
    pt = 0.0

    for w in range(len(soutput)):
        if(soutput[w] == '\x1b[0;38;48m\x1b[0;35;48mPlanning'):
            print soutput[w+2][0:5]
            pt = pt + float(soutput[w+2][0:5])

    return str(pt)

if __name__ == "__main__":

    files = ["screen_output_640am.txt",
             "screen_output_643am.txt",
             "screen_output_647am.txt",
             "screen_output_650am.txt",
             "screen_output_654am.txt",
             "screen_output_658am.txt",
             "screen_output_703am.txt",
             "screen_output_709am.txt",
             "screen_output_717am.txt",
             "screen_output_726am.txt",
             "screen_output_733am.txt",
             "screen_output_741am.txt",
             "screen_output_746am.txt",
             "screen_output_756am.txt",
             "screen_output_803am.txt",
             "screen_output_810am.txt",
             "screen_output_818am.txt"]
    
    
    planning_time_csv_report = open('total_planning_times.csv','w')

    for f in range(len(files)):
        print "total planning time for "+files[f]+" is: "
        print get_total_planning_time(files[f])
        planning_time_csv_report.write(get_total_planning_time(files[f])+'\n')

    planning_time_csv_report.close()
    

#!/usr/bin/python

#   Python script for anaylsis of task monitor output
#   Calder Phillips-Grafflin - WPI DRC Team 2013

def LoadFileIntoList(filename):
    raw_file = open(filename, "r")
    raw_data = raw_file.read()
    raw_file.close()
    raw_chunks = raw_data.split("----------")
    cleaner_chunks = []
    for index in range(1, len(raw_chunks)):
        raw_chunk = raw_chunks[index]
        cleaner_chunk = raw_chunk.lstrip("\n").rstrip("\n")
        cleaner_chunks.append(cleaner_chunk)
    return cleaner_chunks

def ProcessChunk(trajectory_chunk):
    chunk_lines = trajectory_chunk.split("\n")
    processed = {"type":"All bad", "runs":1, "run data":[{"real":"KB","evaluation":"correct"}]}
    runs_data = []
    num_runs = trajectory_chunk.count("evaluation complete")
    #print num_runs
    tagged_codes = []
    for line in chunk_lines:
        if "[INTERACTIVE]" in line:
            code = line.split(": ")[1]
            tagged_codes.append(code)
    #print tagged_codes
    checked = CheckIfSame(tagged_codes)
    #if (checked):
    #    print "All executions produced the same result"
    #else:
    #    print "Not all executions produced the same result - probably a marginal trajectory"
    assigned_codes = []
    for line in chunk_lines:
        if "Best match trajectory" in line:
            code = line.split(": ")[1]
            assigned_codes.append(code)
    #print assigned_codes
    stats = CheckAssignment(tagged_codes, assigned_codes)
    #print "Tagged : " + str(tagged_codes) + " Assigned: " + str(assigned_codes)
    #print "Stats: " + str(stats)
    return stats

def CheckAssignment(tagged, assigned):
    assert(len(tagged) == len(assigned))
    total_good = 0
    total_bad = 0
    total_correct = 0
    total_incorrect = 0
    correct_good = 0
    correct_bad = 0
    false_negative = 0
    false_positive = 0
    for index in range(len(tagged)):
        if (tagged[index] == "KG"):
            total_good += 1
            if (assigned[index] == "KG"):
                correct_good += 1
                total_correct += 1
            elif (assigned[index] == "KB"):
                false_negative += 1
                total_incorrect += 1
            else:
                print "WTF just happened?!"
        elif (tagged[index] == "KB"):
            total_bad += 1
            if (assigned[index] == "KG"):
                false_positive += 1
                total_incorrect += 1
            elif (assigned[index] == "KB"):
                correct_bad += 1
                total_correct += 1
            else:
                print "WTF just happened?!"
        else:
            print "WTF just happened?!"
    quality = CheckIfSame(tagged)
    return {"tg":total_good, "tb":total_bad, "cg":correct_good, "cb":correct_bad, "fp":false_positive, "fn":false_negative, "q":quality}

def CheckIfSame(codes):
    assert(len(codes) > 0)
    start_code = codes[0]
    for code in codes:
        if (code != start_code):
            return False
    return True

def ComputeStats(trajectory_chunks):
    #Analyze the quality trajectories
    combined_quality_stats = []
    total_quality_good = 0
    total_quality_bad = 0
    total_quality_correct_good = 0
    total_quality_correct_bad = 0
    total_quality_false_positive = 0
    total_quality_false_negative = 0
    total_quality_correct = 0
    total_quality_incorrect = 0
    #Analyze the marginal trajectories
    combined_marginal_stats = []
    total_marginal_good = 0
    total_marginal_bad = 0
    total_marginal_correct_good = 0
    total_marginal_correct_bad = 0
    total_marginal_false_positive = 0
    total_marginal_false_negative = 0
    total_marginal_correct = 0
    total_marginal_incorrect = 0
    #Collect data
    for element in trajectory_chunks:
        new_stats = ProcessChunk(element)
        if (new_stats['q']):
            combined_quality_stats.append(new_stats)
            new_correct = new_stats['cg'] + new_stats['cb']
            new_incorrect = new_stats['fp'] + new_stats['fn']
            total_quality_correct += new_correct
            total_quality_incorrect += new_incorrect
            total_quality_good += new_stats['tg']
            total_quality_bad += new_stats['tb']
            total_quality_correct_good += new_stats['cg']
            total_quality_correct_bad += new_stats['cb']
            total_quality_false_positive += new_stats['fp']
            total_quality_false_negative += new_stats['fn']
        else:
            combined_marginal_stats.append(new_stats)
            new_correct = new_stats['cg'] + new_stats['cb']
            new_incorrect = new_stats['fp'] + new_stats['fn']
            total_marginal_correct += new_correct
            total_marginal_incorrect += new_incorrect
            total_marginal_good += new_stats['tg']
            total_marginal_bad += new_stats['tb']
            total_marginal_correct_good += new_stats['cg']
            total_marginal_correct_bad += new_stats['cb']
            total_marginal_false_positive += new_stats['fp']
            total_marginal_false_negative += new_stats['fn']
    #Display stats for quality trajectories
    print "Total (quality) trajectories: " + str(total_quality_good + total_quality_bad)
    print "Total (quality) good trajectories: " + str(total_quality_good)
    print "Total (quality) bad trajectories: " + str(total_quality_bad)
    print "----------"
    print "Total (quality) correctly identified: " + str(total_quality_correct)
    print "Total (quality) incorrectly identified: " + str(total_quality_incorrect)
    print "Total (quality) false positives: " + str(total_quality_false_positive)
    print "Total (quality) false negatives: " + str(total_quality_false_negative)
    print "----------"
    print "Overall (quality) percent identified correctly: " + str((float(total_quality_correct) / float(total_quality_good + total_quality_bad)) * 100.0)
    print "Overall (quality) percent identified incorrectly: " + str((float(total_quality_incorrect) / float(total_quality_good + total_quality_bad)) * 100.0)
    print "Overall (quality) percent false positives: " + str((float(total_quality_false_positive) / float(total_quality_good + total_quality_bad)) * 100.0)
    print "Overall (quality) percent false negatives: " + str((float(total_quality_false_negative) / float(total_quality_good + total_quality_bad)) * 100.0)
    print "Percent (quality) good identified correctly: " + str((float(total_quality_correct_good) / float(total_quality_good)) * 100.0)
    print "Percent (quality) bad identified correctly: " + str((float(total_quality_correct_bad) / float(total_quality_bad)) * 100.0)
    print "Percent (quality) good identified incorrectly: " + str((float(total_quality_false_negative) / float(total_quality_good)) * 100.0)
    print "Percent (quality) bad identified incorrectly: " + str((float(total_quality_false_positive) / float(total_quality_bad)) * 100.0)
    #Display stats for marginal trajectories
    print "===================="
    print "Total (marginal) trajectories: " + str(total_marginal_good + total_marginal_bad)
    print "Total (marginal) good trajectories: " + str(total_marginal_good)
    print "Total (marginal) bad trajectories: " + str(total_marginal_bad)
    print "----------"
    print "Total (marginal) correctly identified: " + str(total_marginal_correct)
    print "Total (marginal) incorrectly identified: " + str(total_marginal_incorrect)
    print "Total (marginal) false positives: " + str(total_marginal_false_positive)
    print "Total (marginal) false negatives: " + str(total_marginal_false_negative)
    print "----------"
    print "Overall (marginal) percent identified correctly: " + str((float(total_marginal_correct) / float(total_marginal_good + total_marginal_bad)) * 100.0)
    print "Overall (marginal) percent identified incorrectly: " + str((float(total_marginal_incorrect) / float(total_marginal_good + total_marginal_bad)) * 100.0)
    print "Overall (marginal) percent false positives: " + str((float(total_marginal_false_positive) / float(total_marginal_good + total_marginal_bad)) * 100.0)
    print "Overall (marginal) percent false negatives: " + str((float(total_marginal_false_negative) / float(total_marginal_good + total_marginal_bad)) * 100.0)
    print "Percent (marginal) good identified correctly: " + str((float(total_marginal_correct_good) / float(total_marginal_good)) * 100.0)
    print "Percent (marginal) bad identified correctly: " + str((float(total_marginal_correct_bad) / float(total_marginal_bad)) * 100.0)
    print "Percent (marginal) good identified incorrectly: " + str((float(total_marginal_false_negative) / float(total_marginal_good)) * 100.0)
    print "Percent (marginal) bad identified incorrectly: " + str((float(total_marginal_false_positive) / float(total_marginal_bad)) * 100.0)

if __name__ == '__main__':
    semiprocessed = LoadFileIntoList("Partial library output.txt")
    print "Processing data gathered from " + str(len(semiprocessed)) + " planned trajectories (" + str(len(semiprocessed) * 3) + " executed)"
    #for part in semiprocessed:
    #    ProcessChunk(part)
    ComputeStats(semiprocessed)

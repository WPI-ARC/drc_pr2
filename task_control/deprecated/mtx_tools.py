#   Statistics generation for an MTX file
#   MTX "dtw Match Trajectory eXecuted"

class MTX:

    def __init__(self, filename):
        raw_file = open(filename, "r")
        raw_data = raw_file.readlines()
        self.matched_pairs = []
        for line in raw_data:
            cleaned_line = line.strip("\n").lstrip("[").rstrip("]")
            chunks = cleaned_line.split(", ")
            pair = []
            for chunk in chunks:
                cleaned_chunk = chunk.lstrip("\'").rstrip("\'")
                pair.append(chunk)
            self.matched_pairs.append(pair)
        print "Loaded MTX file"

    def ExtractCodes(self, raw_codes):
        chunks1 = raw_codes.split(" ")
        cleaned_codes = chunks1[0].split("-")
        fully_cleaned_codes = []
        for code in cleaned_codes:
            cleaned_code = code.lstrip("'").rstrip("'")
            fully_cleaned_codes.append(cleaned_code)
        return fully_cleaned_codes

    def ExactGraspMatch(self):
        grasp_matches = 0
        total = len(self.matched_pairs)
        for pair in self.matched_pairs:
            codes1 = self.ExtractCodes(pair[0])
            codes2 = self.ExtractCodes(pair[1])
            if (codes1[2] == codes2[2] and codes1[3] == codes2[3]):
                grasp_matches = grasp_matches + 1
        print "Computed Exact Grasp Matches:"
        percent_correct = float(grasp_matches) / float(total) * 100.0
        print str(percent_correct) + " percent identified"

    def PartwiseMatch(self):
        force_match = 0
        stop_match = 0
        grasp_match = 0
        alignment_match = 0
        total = len(self.matched_pairs)
        for pair in self.matched_pairs:
            codes1 = self.ExtractCodes(pair[0])
            codes2 = self.ExtractCodes(pair[1])
            if (codes1[0] == codes2[0]):
                force_match = force_match + 1
            if (codes1[1] == codes2[1]):
                stop_match = stop_match + 1
            if (codes1[2] == codes2[2]):
                grasp_match = grasp_match + 1
            if (codes1[3] == codes2[3]):
                alignment_match = alignment_match + 1
        print "Computed Part-wise Matches:"
        percent_correct = float(force_match) / float(total) * 100.0
        print str(percent_correct) + " percent forces identified correctly"
        percent_correct = float(stop_match) / float(total) * 100.0
        print str(percent_correct) + " percent stops identified correctly"
        percent_correct = float(grasp_match) / float(total) * 100.0
        print str(percent_correct) + " percent grasps identified correctly"
        percent_correct = float(alignment_match) / float(total) * 100.0
        print str(percent_correct) + " percent alignments identified correctly"

    def IsGoodGrasp(self, code):
        if (code[2] == 'BG' or code[3] == 'BA'):
            return False
        else:
            return True

    def PartialGraspMatch(self):
        grasp_matches = 0
        bad_count = 0
        good_count = 0
        bad_bad = 0
        bad_good = 0
        good_bad = 0
        good_good = 0
        total = len(self.matched_pairs)
        for pair in self.matched_pairs:
            codes1 = self.ExtractCodes(pair[0])
            codes2 = self.ExtractCodes(pair[1])
            assigned_quality = self.IsGoodGrasp(codes1)
            real_quality = self.IsGoodGrasp(codes2)
            if (real_quality):
                good_count = good_count + 1
            else:
                bad_count = bad_count + 1
            if (real_quality and assigned_quality):
                good_good = good_good + 1
            elif (real_quality and not assigned_quality):
                good_bad = good_bad + 1
            elif (not real_quality and assigned_quality):
                bad_good = bad_good + 1
            else:
                bad_bad = bad_bad + 1
        print "Computed Partial Grasp Matches:"
        percent_correct = float(good_good) / float(good_count) * 100.0
        print str(percent_correct) + " percent good grasps identified correctly"
        percent_correct = float(good_bad) / float(good_count) * 100.0
        print str(percent_correct) + " percent good grasps identified incorrectly"
        percent_correct = float(bad_good) / float(bad_count) * 100.0
        print str(percent_correct) + " percent bad grasps identified incorrectly"
        percent_correct = float(bad_bad) / float(bad_count) * 100.0
        print str(percent_correct) + " percent bad grasps identified correctly"

    def ExactTurningMatch(self):
        turning_matches = 0
        total = len(self.matched_pairs)
        for pair in self.matched_pairs:
            codes1 = self.ExtractCodes(pair[0])
            codes2 = self.ExtractCodes(pair[1])
            if (codes1[0] == codes2[0] and codes1[1] == codes2[1]):
                turning_matches = turning_matches + 1
        print "Computed Exact Turning Matches:"
        percent_correct = float(turning_matches) / float(total) * 100.0
        print str(percent_correct) + " percent identified"

    def IsGoodTurn(self, code):
        if (code[1] == 'AS' or (code[0] == 'HF' and code[1] == 'PS')):
            return False
        else:
            return True

    def PartialTurningMatch(self):
        good_count = 0
        bad_count = 0
        good_turns = 0
        good_good = 0
        good_bad = 0
        bad_good = 0
        bad_bad = 0
        total = len(self.matched_pairs)
        for pair in self.matched_pairs:
            codes1 = self.ExtractCodes(pair[0])
            codes2 = self.ExtractCodes(pair[1])
            assigned_quality = self.IsGoodTurn(codes1)
            real_quality = self.IsGoodTurn(codes2)
            if (real_quality):
                good_count = good_count + 1
            else:
                bad_count = bad_count + 1
            if (real_quality and assigned_quality):
                good_good = good_good + 1
            elif (real_quality and not assigned_quality):
                good_bad = good_bad + 1
            elif (not real_quality and assigned_quality):
                bad_good = bad_good + 1
            else:
                bad_bad = bad_bad + 1
        print "Computed Partial Turning Matches:"
        percent_correct = float(good_good) / float(good_count) * 100.0
        print str(percent_correct) + " percent good turns identified correctly"
        percent_correct = float(good_bad) / float(good_count) * 100.0
        print str(percent_correct) + " percent good turns identified incorrectly"
        percent_correct = float(bad_good) / float(bad_count) * 100.0
        print str(percent_correct) + " percent bad turns identified incorrectly"
        percent_correct = float(bad_bad) / float(bad_count) * 100.0
        print str(percent_correct) + " percent bad turns identified correctly"

    def PerfectMatch(self):
        perfect_matches = 0
        total = len(self.matched_pairs)
        for pair in self.matched_pairs:
            codes1 = self.ExtractCodes(pair[0])
            codes2 = self.ExtractCodes(pair[1])
            if (codes1[0] == codes2[0] and codes1[1] == codes2[1] and codes1[2] == codes2[2] and codes1[3] == codes2[3]):
                perfect_matches = perfect_matches + 1
        print "Computed Perfect Matches:"
        percent_correct = float(perfect_matches) / float(total) * 100.0
        print str(percent_correct) + " percent identified perfectly"

    def NoForceMatch(self):
        no_force_matches = 0
        total = len(self.matched_pairs)
        for pair in self.matched_pairs:
            codes1 = self.ExtractCodes(pair[0])
            codes2 = self.ExtractCodes(pair[1])
            if (codes1[1] == codes2[1] and codes1[2] == codes2[2] and codes1[3] == codes2[3]):
                no_force_matches = no_force_matches + 1
        print "Computed Matches ignoring forces:"
        percent_correct = float(no_force_matches) / float(total) * 100.0
        print str(percent_correct) + " percent identified correctly"

    def AssignmentMatch(self):
        assigned_matches = 0
        total = len(self.matched_pairs)
        for pair in self.matched_pairs:
            codes1 = self.ExtractCodes(pair[0])
            codes2 = self.ExtractCodes(pair[1])
            real_grasp = self.IsGoodGrasp(codes2)
            assigned_grasp = self.IsGoodGrasp(codes1)
            real_turning = self.IsGoodTurn(codes2)
            assigned_turning = self.IsGoodTurn(codes1)
            if (real_grasp == assigned_grasp and real_turning == assigned_turning):
                assigned_matches = assigned_matches + 1
        print "Computed Assignment Matches:"
        percent_correct = float(assigned_matches) / float(total) * 100.0
        print str(percent_correct) + " percent assigned correctly"

    def TunedAssignmentMatch(self):
        assigned_matches = 0
        total = len(self.matched_pairs)
        for pair in self.matched_pairs:
            codes1 = self.ExtractCodes(pair[0])
            codes2 = self.ExtractCodes(pair[1])
            if (codes1[1] == codes2[1] and codes1[2] == codes2[2]):
                assigned_matches = assigned_matches + 1
        print "Computed Tuned Assignment Matches:"
        percent_correct = float(assigned_matches) / float(total) * 100.0
        print str(percent_correct) + " percent assigned correctly"

    def TunedResponseMatch(self):
        response_matches = 0
        total = len(self.matched_pairs)
        for pair in self.matched_pairs:
            codes1 = self.ExtractCodes(pair[0])
            codes2 = self.ExtractCodes(pair[1])
            real_response = self.GenerateTunedResponse(codes2)
            assigned_response = self.GenerateTunedResponse(codes1)
            if (real_response == assigned_response):
                response_matches = response_matches + 1
        print "Computed Tuned Response Matches:"
        percent_correct = float(response_matches) / float(total) * 100.0
        print str(percent_correct) + " percent responded to correctly"

    def TunedUntunedResponseMatch(self):
        response_matches = 0
        total = len(self.matched_pairs)
        for pair in self.matched_pairs:
            codes1 = self.ExtractCodes(pair[0])
            codes2 = self.ExtractCodes(pair[1])
            real_response = self.GenerateResponse(self.IsGoodTurn(codes2), self.IsGoodGrasp(codes2))
            assigned_response = self.GenerateTunedResponse(codes1)
            if (real_response == assigned_response):
                response_matches = response_matches + 1
        print "Computed Tuned Responses Matching Untuned Real:"
        percent_correct = float(response_matches) / float(total) * 100.0
        print str(percent_correct) + " percent responded to correctly"

    def GenerateTunedResponse(self, code):
        if (code[2] == "BG"):
            return "PROMPT"
        elif (code[1] == "PS" or code[1] == "AS"):
            return "REDO"
        else:
            return "CONTINUE"

    def GenerateResponse(self, turn_quality, grasp_quality):
        if (grasp_quality == False):
            return "PROMPT"
        elif (turn_quality == False):
            return "REDO"
        else:
            return "CONTINUE"

    def ResponseMatch(self):
        response_matches = 0
        total = len(self.matched_pairs)
        for pair in self.matched_pairs:
            codes1 = self.ExtractCodes(pair[0])
            codes2 = self.ExtractCodes(pair[1])
            real_grasp = self.IsGoodGrasp(codes2)
            assigned_grasp = self.IsGoodGrasp(codes1)
            real_turning = self.IsGoodTurn(codes2)
            assigned_turning = self.IsGoodTurn(codes1)
            real_response = self.GenerateResponse(real_turning, real_grasp)
            assigned_response = self.GenerateResponse(assigned_turning, assigned_grasp)
            if (real_response == assigned_response):
                response_matches = response_matches + 1
        print "Computed Response Matches:"
        percent_correct = float(response_matches) / float(total) * 100.0
        print str(percent_correct) + " percent responded to correctly"

if __name__ == '__main__':
    filename1 = "basic_match.mtx"
    mtx_evaluator1 = MTX(filename1)
    print "Joint data analysis"
    mtx_evaluator1.PerfectMatch()
    mtx_evaluator1.NoForceMatch()
    mtx_evaluator1.PartwiseMatch()
    mtx_evaluator1.ExactGraspMatch()
    mtx_evaluator1.PartialGraspMatch()
    mtx_evaluator1.ExactTurningMatch()
    mtx_evaluator1.PartialTurningMatch()
    mtx_evaluator1.AssignmentMatch()
    mtx_evaluator1.TunedAssignmentMatch()
    mtx_evaluator1.ResponseMatch()
    mtx_evaluator1.TunedResponseMatch()
    mtx_evaluator1.TunedUntunedResponseMatch()
    print "*************************"
    filename2 = "pose_match.mtx"
    mtx_evaluator2 = MTX(filename2)
    print "Pose data analysis"
    mtx_evaluator2.PerfectMatch()
    mtx_evaluator2.NoForceMatch()
    mtx_evaluator2.PartwiseMatch()
    mtx_evaluator2.ExactGraspMatch()
    mtx_evaluator2.PartialGraspMatch()
    mtx_evaluator2.ExactTurningMatch()
    mtx_evaluator2.PartialTurningMatch()
    mtx_evaluator2.AssignmentMatch()
    mtx_evaluator2.TunedAssignmentMatch()
    mtx_evaluator2.ResponseMatch()
    mtx_evaluator2.TunedResponseMatch()
    mtx_evaluator2.TunedUntunedResponseMatch()
    print "*************************"
    filename3 = "pose_match_selected.mtx"
    mtx_evaluator3 = MTX(filename3)
    print "Selected pose data analysis"
    mtx_evaluator3.PerfectMatch()
    mtx_evaluator3.NoForceMatch()
    mtx_evaluator3.PartwiseMatch()
    mtx_evaluator3.ExactGraspMatch()
    mtx_evaluator3.PartialGraspMatch()
    mtx_evaluator3.ExactTurningMatch()
    mtx_evaluator3.PartialTurningMatch()
    mtx_evaluator3.AssignmentMatch()
    mtx_evaluator3.TunedAssignmentMatch()
    mtx_evaluator3.ResponseMatch()
    mtx_evaluator3.TunedResponseMatch()
    mtx_evaluator3.TunedUntunedResponseMatch()

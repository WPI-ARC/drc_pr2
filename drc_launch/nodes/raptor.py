#!/usr/bin/python

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    QUESTION = '\033[90m'
    FAIL = '\033[91m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    ENDC = '\033[0m'

if __name__ == '__main__':
    raptor_string = bcolors.BOLD + bcolors.CYAN + "                                                        .--.__\n                                                      .~ (@)  ~~~---_\n                                                     {     `-_~,,,,,,)\n                                                     {    (_  ',\n                                                      ~    . = _',\n                                                       ~-   '.  =-'\n                                                         ~     :\n      .                                             _,.-~     ('');\n      '.                                         .-~        \  \ ;\n        ':-_                                _.--~            \  \;      _-=,.\n          ~-:-.__                       _.-~                 {  '---- _'-=,.\n             ~-._~--._             __.-~    BAMBIRAPTOR      ~---------=,.`\n                 ~~-._~~-----~~~~~~       .+++~~~~~~~~-__   /\n                      ~-.,____           {   -     +   }  _/\n                              ~~-.______{_    _ -=\ / /_.~\nNick Alunni                        :      ~--~    // /         ..-\nCalder Phillips-Grafflin           :   / /      // /         ((\nBener Suay                         :  / /      {   `-------,. ))\nDmitry Berenson                    :   /        ''=--------. }o\nSonia Chernova        .=._________,'  )                     ))\nRob Lindeman          )  _________ -''                     ~~\nWPI DRC 2013         / /  _ _\nVersion 0.2         (_.-.'O'-'." + bcolors.ENDC
    border_string = bcolors.BOLD + bcolors.FAIL + "=============================================================================" + bcolors.ENDC
    print border_string
    print raptor_string
    print border_string

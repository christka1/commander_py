#   -------------------------------------------------------------------------------
#   Description:
#   Creates and appends a daily logfile.
#   Call method write_to_log to write to the daily log-file.
#   If the file does not exist it creates a new one.
#   -------------------------------------------------------------------------------
#   Author: Sebastian Brink
#   Kandidatarbete COBOTS 2019
#   V.1.0.0
#   -------------------------------------------------------------------------------

import datetime
import time
import os
from pathlib import Path

# Skapa en metod för allt, kolla först om mappen finns, sen om filen finns,
#  sen skriv till filen
# jämför med nuvarande datum för att skapa fil.


def write_to_log(msg: str):
    # check if the log folder exists, if not then create it
    dirname = os.path.dirname(__file__)
    # print(dirname)
    relativedirname = os.path.join(dirname, '/logs/')

    currentDate = datetime.date.today()
    # print(currentDate)
    log_dir = Path("{}{}".format(dirname, relativedirname))
    # log_dir = Path(dirname+'/logs/')
    if not log_dir.exists():
        os.makedirs(log_dir)

    # find the path to todays log and check if it exists, if not
    # create it

    my_file = Path("{}{}{}{}".format(
        dirname, relativedirname, currentDate, '.txt'))

    # if not my_file.exists():
    #    print('hejhej')
    #    f = open(my_file, 'w+')

    currentTime = datetime.datetime.now().strftime('%H:%M:%S')

    # open todays logfile and begin writing to it
    with open(my_file, "a") as log:
        log.write('[{}] \t {} \n'.format(currentTime, msg))


# write_to_log('test1')

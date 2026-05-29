import os
import sys
from datetime import datetime


projectFolder = os.getcwd()
versionFile = os.path.join(projectFolder, 'src', '__version.h')
tmpFile = os.path.join(projectFolder, 'src', 'tmpver.h')

if not os.path.isfile(versionFile):
    print(versionFile)
    sys.exit('ERROR: file __version.h does not exist!!')

f = open(versionFile)
d = open(tmpFile, 'w+')

now = datetime.now()

for l in f:
    if 'DATETIMESTR' in l:
        d.write('#define DATETIMESTR \t\"%s\"\n' % now.strftime("%a %b %Y %H:%M:%S"))        
    elif 'DATETIME' in l:
        d.write('#define DATETIME \t\"%s\"\n' % now.strftime("%Y%m%d%H%M%S"))
    elif 'BUILD' in l:
        ss = l.split()
        build=int(ss[2].replace('"', ''))
        d.write('#define BUILD \t\t\"%s\"\n' % (build+1))
        print('build %s' % (build+1))
    else:
        d.write(l)
        
d.close()
f.close()
os.remove(versionFile)
os.rename(tmpFile, versionFile)
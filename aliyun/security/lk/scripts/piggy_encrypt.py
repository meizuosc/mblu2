#!/user/bin/env python
import sys,os

def main():
    infile = sys.argv[1]
    outfile = sys.argv[2]
    print "infile:", infile
    print "outfile:", outfile

    os.system("echo %s"%infile)
    piggy_size = os.path.getsize(infile)
    print "piggy_size:", piggy_size

    os.system("dd bs=1 count=10 skip=0 if=%s of=%s.head"%(infile, infile))
    os.system("dd bs=1 count=512 skip=10 if=%s of=%s.body"%(infile, infile))
    os.system("dd bs=522 skip=1 if=%s of=%s.tail"%(infile, infile))

    os.system("openssl enc -aes-128-cbc -nosalt -nopad -in %s.body -out %s.body.enc -K 594D645A6A6725798325325E516B5A8F -iv 7C20732D252C7B69258032E079679A4D"%(infile, infile))
    os.system("cat %s.head %s.body.enc %s.tail > %s.tmp"%(infile, infile, infile, infile))
    os.system("mv %s.tmp %s"%(infile, outfile))
    os.system("rm %s.head %s.body %s.body.enc %s.tail"%(infile, infile, infile, infile))

if __name__ == '__main__':
    print "-------- piggy encrypt now --------"
    main()
    print "-------- piggy encrypt OK --------"

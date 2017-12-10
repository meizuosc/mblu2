#!/user/bin/env python
import sys,os

def main():
    infile = sys.argv[1]
    print "infile:", infile

    os.system("echo %s"%infile)
    image_size = os.path.getsize(infile)
    body_size = image_size - 512
    print "image_size:", image_size
    print "body_size:", body_size

    image_blocks = image_size // 512
    body_blocks = body_size // 512
    print "image_blocks:",  image_blocks
    print "body_blocks:", body_blocks

    os.system("dd bs=512 count=1 skip=0 if=%s of=%s.head"%(infile, infile))
    os.system("dd bs=512 count=%s skip=1 if=%s of=%s.body"%(body_blocks, infile, infile))
    os.system("dd bs=512 skip=%s if=%s of=%s.tail"%(image_blocks, infile, infile))

    os.system("openssl enc -aes-128-cbc -nosalt -nopad -in %s.body -out %s.body.enc -K 594D645A6A6725798325325E516B5A8F -iv 7C20732D252C7B69258032E079679A4D"%(infile, infile))
    os.system("cat %s.head %s.body.enc %s.tail > %s.enc"%(infile, infile, infile, infile))
    os.system("mv %s.enc %s"%(infile, infile))
    os.system("rm %s.head %s.body %s.body.enc %s.tail"%(infile, infile, infile, infile))

if __name__ == '__main__':
    print "-------- ramdisk.img encrypt now --------"
    main()
    print "-------- ramdisk.img encrypt OK --------"

import boto
import boto.s3
import sys
from boto.s3.key import Key

if len(sys.argv) != 4:
    print 'Usage: upload.py filename AWS_ACCESS_KEY_ID AWS_SECRET_ACCESS_KEY'
    sys.exit(1)

filename = str(sys.argv[1])
AWS_ACCESS_KEY_ID = str(sys.argv[2])
AWS_SECRET_ACCESS_KEY = str(sys.argv[3])

bucket_name = 'priusdata'
conn = boto.connect_s3(AWS_ACCESS_KEY_ID,
        AWS_SECRET_ACCESS_KEY)


bucket = conn.get_bucket(bucket_name)
#    location=boto.s3.connection.Location.DEFAULT)

print 'Uploading %s to Amazon S3 bucket %s' % \
   (filename, bucket_name)

def percent_cb(complete, total):
    sys.stdout.write('.')
    sys.stdout.flush()

k = Key(bucket)
k.key = 'prius_data.txt'
k.set_contents_from_filename(filename,
    cb=percent_cb, num_cb=10)

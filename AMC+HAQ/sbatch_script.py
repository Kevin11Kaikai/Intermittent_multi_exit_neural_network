
def deleteLines():
    bad_words = ['Max packets:', 'mean episode reward:']

    fileName = ['testfile.txt']

    with open(fileName[0], "r") as f:
        lines = f.readlines()
    with open(fileName[0], "w") as f:
        i = 1
        for line in lines:
            # if line.strip("\n") != "nickname_to_delete":
            if not any(bad_word in line for bad_word in bad_words):
                f.write(str(i)+' '+line)
                i += 1

# with open('oldfile.txt') as oldfile, open('newfile.txt', 'w') as newfile:
#     for line in oldfile:
#         if not any(bad_word in line for bad_word in bad_words):
#             newfile.write(line)


def editLines(target_size, preserve_ratio):
    fileName = ['job.slurm']

    with open(fileName[0], "r") as f:
        lines = f.readlines()
    with open('sbatch_input.txt', "w") as f:
        for line in lines:
            if line.startswith('python'):
                # print(line)
                str0 = line.split()
                # print(str)
                for i, eachItem in enumerate(str0):
                    # print(i, eachItem)
                    if eachItem.startswith('--target_mode_size'):
                        #print("Found:", eachItem)
                        str0[i] = '--target_mode_size='+str(target_size)

                    # elif eachItem.startswith('--num-units'):
                    #     #print("Found:", eachItem)
                    #     str0[i] = '--num-units='+str(units)
                    if eachItem.startswith('--preserve_ratio') and (not eachItem.startswith('--preserve_ratio2')):
                        #print("Found:", eachItem)
                        str0[i] = '--preserve_ratio='+str(preserve_ratio)

                # join each item
                str1 = ''
                for item in str0:
                    str1 += item
                    str1 += ' '

                print(str1)

                f.write(str1)
            else:
                f.write(line)


def runcmd(cmd):
    import subprocess
    # result = subprocess.run(['ls', '-l'], stdout=subprocess.PIPE)
    # result = subprocess.run(cmd, stdout=subprocess.PIPE)

    output = subprocess.Popen(cmd, stdout=subprocess.PIPE).communicate()[0]

    #result_string = result.stdout.decode('utf-8')
    print("result\n", output)

def run_command(command):
    import subprocess
    p = subprocess.Popen(command,
                         stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT)
    return iter(p.stdout.readline, b'')

def run_cmd(cmd, arg):
    import subprocess
    #p = subprocess.Popen(['ls', '-a'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    p = subprocess.Popen([cmd, arg], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    out, err = p.communicate()
    print out
    

def runbatch():
    preserve_ratio = [0.5, 0.75]
    target_size = [8192, 16384, 32768, 49152, 65536, 131072]

    for ratio in preserve_ratio:
        for size in target_size:
            editLines(target_size=size, preserve_ratio=ratio)
            #run_command('sbatch ./output.txt')
            #print(run_command('ls'))
            run_cmd('sbatch', 'sbatch_input.txt')


runbatch()
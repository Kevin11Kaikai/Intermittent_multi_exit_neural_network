from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import h5py



# import sys
import os
import torch
import argparse
# import data
# import util
import torch.nn as nn
# import torch.optim as optim

# from models import nin
from torch.autograd import Variable

import torch.nn.functional as F

# def save_state(model, best_acc):
#     print('==> Saving model ...')
#     state = {
#             'best_acc': best_acc,
#             'state_dict': model.state_dict(),
#             }
#     for key in state['state_dict'].keys():
#         if 'module' in key:
#             state['state_dict'][key.replace('module.', '')] = \
#                     state['state_dict'].pop(key)
#     torch.save(state, 'models/nin.pth.tar')

def save_state(model, acc):#function for saving the state with model and acc as the input
    print('==> Saving model ...')#print('==> Saving model ...')
    state = {
            'acc': acc,
            'state_dict': model.state_dict(),
            }#state is a dictionary that contains 'acc': acc, 'state_dict': model.state_dict()
    for key in state['state_dict'].keys():
        if 'module' in key:
            state['state_dict'][key.replace('module.', '')] = \
                    state['state_dict'].pop(key)
    torch.save(state, 'models/'+args.arch+'.best.pth.tar')#save the best path

def train(epoch):#function for training the model with 'epoch' as the input
    model.train()
    for batch_idx, (data, target) in enumerate(trainloader): #iterate the train loader
        # process the weights including binarization
        # bin_op.binarization()
        
        # forwarding
        data, target = Variable(data.cuda()), Variable(target.cuda())#transform data, target into data.cuda(), target.cuda() for GPU acceleration
        optimizer.zero_grad()#zero the gradient
        output = model(data)#get the output from the data
        
        # backwarding
        # loss = criterion(output, target)

        if not isinstance(output, list):
            output = [output]
        loss = 0.0
        for j in range(len(output)):
            loss += criterion(output[j], target)

        loss.backward()#backward the loss
        
        # restore weights
        # bin_op.restore()
        # bin_op.updateBinaryGradWeight()
        
        optimizer.step()#step the optimizer
        if batch_idx % 100 == 0:
            print('Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}\tLR: {}'.format(
                epoch, batch_idx * len(data), len(trainloader.dataset),
                100. * batch_idx / len(trainloader), loss.data.item(),
                optimizer.param_groups[0]['lr']))# print Train Epoch: epoch [barch_idx*len(data)/len(trainloader.dataset)]
                #\tLoss: 100. *batch_index / len(trainloader)\tLR: loss.data.item()
    return#Return nothing

def test(epoch = 1):#The function for testing with no input
    global best_acc#set the best_acc as the global variable
    model.eval()#evaluate the model
    test_loss = 0#initiate the test_loss as 0
    # correct = 0
    correct = [0, 0, 0]#initiate correct as [0, 0, 0]
    # bin_op.binarization()
    for data, target in testloader:#iterate the testloader
        data, target = Variable(data.cuda()), Variable(target.cuda()) #transform data and target for GPU acceleration
                                    
        output = model(data)#get the output

        if not isinstance(output, list):
            output = [output]
        loss = 0.0
        for j in range(len(output)):
            test_loss += criterion(output[j], target).data.item() #get the test loss

        # pred = output.data.max(1, keepdim=True)[1]
        # correct += pred.eq(target.data.view_as(pred)).cpu().sum()

        for j in range(len(output)):
            pred = output[j].argmax(dim=1, keepdim=True)  # get the index of the max log-probability
            correct[j] += pred.eq(target.view_as(pred)).sum().item()#get the number of correct predictions 

        # test_loss += criterion(output, target).data.item()
        # pred = output.data.max(1, keepdim=True)[1]
        # correct += pred.eq(target.data.view_as(pred)).cpu().sum()
    # bin_op.restore()
    acc = 100. * float(correct[-1]) / len(testloader.dataset)# get the accuracy

    if acc > best_acc:
        best_acc = acc
        # save_state(model, best_acc)

        print('Saving..')
        #state = {
        #    'net': model.state_dict(),
        #    'acc': acc,
        #    'epoch': epoch,
        #}
        #if not os.path.isdir('checkpoint'):
        #    os.mkdir('checkpoint')
        #torch.save(state, './checkpoint/ckpt.pth')
        # Train the model on GPU
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        model.to(device)
        # Save the trained PyTorch model as a .pt file
        torch.save(model.state_dict(), 'model.pt')
        # Convert the .pt file to an HDF5 file
        h5file = h5py.File('model.h5', 'w')
        for name, param in model.named_parameters():
            param_cpu = param.detach().cpu().numpy()
            h5file.create_dataset(name, data=param_cpu)
        h5file.close()

        
        best_acc = acc#save the one with the best accuracy

    test_loss /= len(testloader.dataset)
    print('\nTest set: Average loss: {:.4f}, Accuracy: {}/{} ({:.2f}%), {}/{} ({:.2f}%), {}/{} ({:.2f}%)'.format(
        test_loss * 100,
        correct[0], len(testloader.dataset), 100. * float(correct[0]) / len(testloader.dataset),
        correct[1], len(testloader.dataset), 100. * float(correct[1]) / len(testloader.dataset),
        correct[2], len(testloader.dataset), 100. * float(correct[2]) / len(testloader.dataset)
        ))
    print('Best Accuracy: {:.2f}%\n'.format(best_acc))
    return
####
#basic functions like save_state, train, and test the network
####
class LeNetEarlyExit2_2(nn.Module):
    def __init__(self):
        super(LeNetEarlyExit2_2, self).__init__()
        # For mnist, 28x28
        # self.conv1 = nn.Conv2d(1, 5, 5, stride=1, padding=3)
        # For Cifar-10, 32x32
        self.conv1 = nn.Conv2d(3, 5, 5, stride=1, padding=1)

        self.conv_b1 = nn.Conv2d(5, 10, 3, stride=1, padding=1)
        self.conv2 = nn.Conv2d(5, 10, 5, stride=1, padding=3)
        self.conv3 = nn.Conv2d(10, 20, 5, stride=1, padding=3)

        self.conv_b2 = nn.Conv2d(10, 20, 5, stride=1, padding=3)
        self.conv_b22 = nn.Conv2d(20, 20, 3, stride=1, padding=1)

        self.fc_exit0 = nn.Linear(640, 10)
        self.fc_exit1 = nn.Linear(720, 84)
        self.fc_exit11 = nn.Linear(84, 10)

        self.fc1 = nn.Linear(720, 84)
        self.fc2 = nn.Linear(84, 10)

    def forward(self, x):
        res = []

        out = self.conv1(x)

        # print(out.shape)

        exit0 = F.relu(F.max_pool2d(out, 2))
        # print(exit0.shape)
        # Branch 1
        exit0 = self.conv_b1(exit0)
        # print(exit0.shape)
        exit0 = F.relu(F.max_pool2d(exit0, 2, padding=1))
        # print(exit0.shape)
        exit0 = exit0.view(exit0.size(0), -1)
        # print(exit0.shape)
        exit0 = self.fc_exit0(exit0)
        # exit0 = F.log_softmax(exit0, dim=1)

        out = F.relu(F.max_pool2d(out, 2))

        out = self.conv2(out)
        out = F.max_pool2d(out, 2)
        out = F.relu(out)

        # Branch 2
        exit1 = self.conv_b2(out)
        # print(exit1.shape)
        # exit1 = F.relu(F.max_pool2d(exit1, 2, padding=1))
        exit1 = F.relu(exit1)
        exit1 = self.conv_b22(exit1)
        # print(exit1.shape)
        exit1 = F.relu(F.max_pool2d(exit1, 2, padding=1))
        # print(exit1.shape)
        exit1 = exit1.view(exit1.size(0), -1)
        # print(exit1.shape)
        exit1 = self.fc_exit1(exit1)
        exit1 = self.fc_exit11(exit1)

        # Main exit
        out = self.conv3(out)
        # print(out.shape)
        out = F.relu(F.max_pool2d(out, 2, padding=1))
        # print(out.shape)
        out = out.view(out.size(0), -1)
        out = self.fc1(out)
        out = self.fc2(out)
        # out = F.log_softmax(out, dim=1)

        res.append(exit0)
        res.append(out)
        res.append(exit1)

        return res

if __name__=='__main__':
    # prepare the options
    parser = argparse.ArgumentParser()#set the parser
    parser.add_argument('--cpu', action='store_true',
            help='set if only CPU is available')#set if only CPU is available
    parser.add_argument('--data', action='store', default='./data/',
            help='dataset path')#Get the dataset path
    parser.add_argument('--arch', action='store', default='nin',
            help='the architecture for the network: nin')# set the architecture for the network
    parser.add_argument('--lr', action='store', default='0.1',
            help='the intial learning rate')#Get the initial learning rate
    parser.add_argument('--pretrained', action='store', default=None,
            help='the path to the pretrained model')# Get the path to the pretrained model
    parser.add_argument('--evaluate', action='store_true',
            help='evaluate the model')# Evaluate the model
    parser.add_argument('--nnoption', action='store', default='0',
                        help='nn option')
    args = parser.parse_args()
    print('==> Options:',args)

    import torchvision
    import torchvision.transforms as transforms
    # Data
    print('==> Preparing data..')
    transform_train = transforms.Compose([
        transforms.RandomCrop(32, padding=4),
        transforms.RandomHorizontalFlip(),
        transforms.ToTensor(),
        transforms.Normalize((0.4914, 0.4822, 0.4465), (0.2023, 0.1994, 0.2010)),
    ])#Normalize data

    transform_test = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.4914, 0.4822, 0.4465), (0.2023, 0.1994, 0.2010)),
    ])#Normailze the test data

    trainset = torchvision.datasets.CIFAR10(root='./data', train=True, download=True, transform=transform_train)
    #Get the trainset
    trainloader = torch.utils.data.DataLoader(trainset, batch_size=128, shuffle=True, num_workers=2)
    #load the data into trainloader
    testset = torchvision.datasets.CIFAR10(root='./data', train=False, download=True, transform=transform_test)
    #Get the test dataset
    testloader = torch.utils.data.DataLoader(testset, batch_size=100, shuffle=False, num_workers=2)
    #Get the test dataset loader
    # define the model
    # print('==> building model',args.arch,'...')
    # if args.arch == 'nin':
    #     model = nin.Net()
    # else:
    #     raise Exception(args.arch+' is currently not supported')

    model = LeNetEarlyExit2_2()#model is the predefined model

    # initialize the model
    if not args.pretrained:
        print('==> Initializing model parameters ...')
        best_acc = 0
        for m in model.modules():
            if isinstance(m, nn.Conv2d):
                m.weight.data.normal_(0, 0.05)
                m.bias.data.zero_()
    else:
        print('==> Load pretrained model form', args.pretrained, '...')
        pretrained_model = torch.load(args.pretrained)
        best_acc = pretrained_model['best_acc']
        model.load_state_dict(pretrained_model['state_dict'])

    #if not args.cpu:
    #    model.cuda()
    #    model = torch.nn.DataParallel(model, device_ids=range(torch.cuda.device_count()))
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model.to(device)
    if torch.cuda.device_count() > 1:
        model = torch.nn.DataParallel(model)

    print(model)

    # define solver and criterion
    base_lr = float(args.lr)
    param_dict = dict(model.named_parameters())
    params = []

    for key, value in param_dict.items():
        params += [{'params':[value], 'lr': base_lr,
            'weight_decay':0.00001}]

    # optimizer = optim.Adam(params)
    # For LeNet-Branchy with mnist, Cifar-10
    optimizer = torch.optim.SGD(model.parameters(), lr=0.01, momentum=0.5)
    #  SGD with momentum = 0.5
    criterion = nn.CrossEntropyLoss()
    #Use the Cross Entropy Loss
    # define the binarization operator
    # bin_op = util.BinOp(model)

    # do the evaluation if specified
    if args.evaluate:
        test()
        exit(0)

    # start training
    for epoch in range(1, 320):#Train for 320 epochs
        train(epoch)
        test(epoch)
        

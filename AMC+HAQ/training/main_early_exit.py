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

from copy import deepcopy
import torchvision
import torchvision.transforms as transforms

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

def save_state(model, acc):
    print('==> Saving model ...')
    state = {
            'acc': acc,
            'state_dict': model.state_dict(),
            }
    for key in state['state_dict'].keys():
        if 'module' in key:
            state['state_dict'][key.replace('module.', '')] = \
                    state['state_dict'].pop(key)
    torch.save(state, 'models/'+args.arch+'.best.pth.tar')

def train(epoch, model, trainloader, optimizer, criterion):
    model.train()
    for batch_idx, (data, target) in enumerate(trainloader):
        # process the weights including binarization
        # bin_op.binarization()
        
        # forwarding
        data, target = Variable(data.cuda()), Variable(target.cuda())
        optimizer.zero_grad()
        output = model(data)
        
        # backwarding
        # loss = criterion(output, target)

        if not isinstance(output, list):
            output = [output]
        loss = 0.0
        for j in range(len(output)):
            loss += criterion(output[j], target)

        loss.backward()
        
        # restore weights
        # bin_op.restore()
        # bin_op.updateBinaryGradWeight()
        
        optimizer.step()
        if batch_idx % 100 == 0:
            print('Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}\tLR: {}'.format(
                epoch, batch_idx * len(data), len(trainloader.dataset),
                100. * batch_idx / len(trainloader), loss.data.item(),
                optimizer.param_groups[0]['lr']))
    return

def test(model, testloader, criterion):
    global best_acc
    model.eval()
    test_loss = 0
    # correct = 0
    correct = [0, 0, 0]
    # bin_op.binarization()
    for data, target in testloader:
        data, target = Variable(data.cuda()), Variable(target.cuda())
                                    
        output = model(data)

        if not isinstance(output, list):
            output = [output]
        loss = 0.0
        for j in range(len(output)):
            test_loss += criterion(output[j], target).data.item()

        # pred = output.data.max(1, keepdim=True)[1]
        # correct += pred.eq(target.data.view_as(pred)).cpu().sum()

        for j in range(len(output)):
            pred = output[j].argmax(dim=1, keepdim=True)  # get the index of the max log-probability
            correct[j] += pred.eq(target.view_as(pred)).sum().item()

        # test_loss += criterion(output, target).data.item()
        # pred = output.data.max(1, keepdim=True)[1]
        # correct += pred.eq(target.data.view_as(pred)).cpu().sum()
    # bin_op.restore()
    acc = 100. * float(correct[-1]) / len(testloader.dataset)

    if acc > best_acc:
        best_acc = acc
        # save_state(model, best_acc)

        print('Saving..')
        state = {
            'net': model.state_dict(),
            'acc': acc,
            # 'epoch': epoch,
        }
        if not os.path.isdir('checkpoint'):
            os.mkdir('checkpoint')
        torch.save(state, './checkpoint/ckpt.pth')
        best_acc = acc

    test_loss /= len(testloader.dataset)
    print('\nTest set: Average loss: {:.4f}, Accuracy: {}/{} ({:.2f}%), {}/{} ({:.2f}%), {}/{} ({:.2f}%)'.format(
        test_loss * 100,
        correct[0], len(testloader.dataset), 100. * float(correct[0]) / len(testloader.dataset),
        correct[1], len(testloader.dataset), 100. * float(correct[1]) / len(testloader.dataset),
        correct[2], len(testloader.dataset), 100. * float(correct[2]) / len(testloader.dataset)
        ))
    print('Best Accuracy: {:.2f}%\n'.format(best_acc))
    return

def adjust_learning_rate_for_pruned(optimizer, epoch):
    update_list = [120, 200, 240, 280]
    if epoch in update_list:
        for param_group in optimizer.param_groups:
            param_group['lr'] = param_group['lr'] * 0.1
    return

class LeNetEarlyExit2_2_CFG(nn.Module):
    def __init__(self, prune_d_list, prune_strategy):
        super(LeNetEarlyExit2_2_CFG, self).__init__()

        self.channels = deepcopy(prune_d_list)
        self.policy = deepcopy(prune_strategy)

        print("input channels:", self.channels)
        print("reserve ratio:", self.policy)

        self.channels.insert(0, 0)
        self.channels.insert(0, 0)
        self.policy.insert(0, 0)
        self.policy.insert(0, 0)
        # The input channel requirements: in3 = in4, in5 = in6
        # 2
        self.conv1 = nn.Conv2d(self.channels[2], self.channels[3], 5, stride=1, padding=1)
        # 3
        self.conv_b1 = nn.Conv2d(self.channels[3], int(10*self.policy[8]), 3, stride=1, padding=1)
        # 4
        self.conv2 = nn.Conv2d(self.channels[4], self.channels[5], 5, stride=1, padding=3)
        # 5
        self.conv3 = nn.Conv2d(self.channels[5], int(20*self.policy[11]), 5, stride=1, padding=3)
        # 6 ???? The # in channels is 8, but # in channels of node 5 is 10
        self.conv_b2 = nn.Conv2d(self.channels[6], self.channels[7], 5, stride=1, padding=3)
        # 7
        self.conv_b22 = nn.Conv2d(self.channels[7], int(20*self.policy[9]), 3, stride=1, padding=1)
        # 8
        self.fc_exit0 = nn.Linear(self.channels[8], 10)
        # 9
        self.fc_exit1 = nn.Linear(self.channels[9], self.channels[10])
        # 10
        self.fc_exit11 = nn.Linear(self.channels[10], 10)
        # 11
        self.fc1 = nn.Linear(self.channels[11], self.channels[12])
        # 12
        self.fc2 = nn.Linear(self.channels[12], 10)


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

def train_pruned_model(prune_strategy, prune_d_prime_list):
    # prepare the options
    print("Starting training the pruned model")
    # set the seed
    # torch.manual_seed(1)
    # torch.cuda.manual_seed(1)

    # Data
    print('==> Preparing data..')
    transform_train = transforms.Compose([
        transforms.RandomCrop(32, padding=4),
        transforms.RandomHorizontalFlip(),
        transforms.ToTensor(),
        transforms.Normalize((0.4914, 0.4822, 0.4465), (0.2023, 0.1994, 0.2010)),
    ])

    transform_test = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.4914, 0.4822, 0.4465), (0.2023, 0.1994, 0.2010)),
    ])

    trainset = torchvision.datasets.CIFAR10(root='./data', train=True, download=True, transform=transform_train)
    trainloader = torch.utils.data.DataLoader(trainset, batch_size=128, shuffle=True, num_workers=2)

    testset = torchvision.datasets.CIFAR10(root='./data', train=False, download=True, transform=transform_test)
    testloader = torch.utils.data.DataLoader(testset, batch_size=100, shuffle=False, num_workers=2)

    # define the model
    # print('==> building model',args.arch,'...')
    # if args.arch == 'nin':
    #     model = nin.Net()
    # else:
    #     raise Exception(args.arch+' is currently not supported')

    # model = LeNetEarlyExit()
    # model = LeNetEarlyExit2()

    model = LeNetEarlyExit2_2_CFG(prune_d_prime_list, prune_strategy)

    # initialize the model
    print('==> Initializing model parameters ...')
    for m in model.modules():
        if isinstance(m, nn.Conv2d):
            m.weight.data.normal_(0, 0.05)
            m.bias.data.zero_()

    model.cuda()
    model = torch.nn.DataParallel(model, device_ids=range(torch.cuda.device_count()))
    print(model)

    # define solver and criterion
    # param_dict = dict(model.named_parameters())

    # optimizer = optim.Adam(params)
    # For LeNet-Branchy with mnist, Cifar-10
    optimizer = torch.optim.SGD(model.parameters(), lr=0.01, momentum=0.5)

    criterion = nn.CrossEntropyLoss()

    # define the binarization operator
    # bin_op = util.BinOp(model)

    # do the evaluation if specified
    # if args.evaluate:
    #     test()
    #     exit(0)

    # start training
    for epoch in range(1, 320):
        adjust_learning_rate_for_pruned(optimizer, epoch)
        train(epoch, model, trainloader, optimizer, criterion)
        test(model, testloader, criterion)

    return model

best_acc = 0
# if __name__ == '__main__':
#     train_pruned_model([1.0, 0.8, 0.8, 0.8, 0.8, 0.6, 0.8, 0.7, 0.5476190476190477, 0.7, 0.21428571428571427], [3, 4, 4, 8, 8, 12, 512, 504, 46, 504, 18])
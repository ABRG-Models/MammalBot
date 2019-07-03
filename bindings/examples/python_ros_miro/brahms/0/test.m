
% this is a simple test script for your new process

function test

% empty system
sys = sml_system;

% add process
state = [];
fS = 10;
cls = '';
sys = sys.addprocess('process', 'examples/python_ros', fS, state);

% execution
exe = brahms_execution;
exe.all = true;
exe.name = 'process';
exe.stop = 1;

% run brahms
[out, rep] = brahms(sys, exe);

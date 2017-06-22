% load textfile
arg_list=argv();
fin=arg_list{1};

data=load(fin);

% plot data
hax1=subplot(2,1,1);
plot(data(:,1),data(:,2:5));
grid('on');
xlabel('t [s]'); ylabel('pixels');
legend({'ul','vl','ur','vr'});

hax2=subplot(2,1,2);
plot(data(:,1),data(:,9:11));
grid('on');
xlabel('t [s]'); ylabel('encoders [deg]');
legend({'tilt','pan','ver'});

linkaxes([hax1 hax2],'x');

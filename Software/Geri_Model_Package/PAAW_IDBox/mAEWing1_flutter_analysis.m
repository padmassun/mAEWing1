function outs = mAEWing1_flutter_analysis(Vrng,nVs,Vmps,coeffsIC,coeffsPID,AEC6,ModeShape,FEM,aeroProp,massProp,chkrng,Vguess)

%analyze pole migration and flutter points between 2 models

%get mode migration and flutter point -------------------------------------
Vlo = Vrng(1);
Vhi = Vrng(2);
Vs = linspace(Vlo,Vhi,nVs);

% opts.mac = 'trad';
% opts.realimag = 0;
opts.mac = 'macx';
opts.realimag = 1;

Afunc = @(Vmps)Ndof6_FDFLEX(AEC6,ModeShape,FEM,Vmps,aeroProp,massProp,coeffsPID);
[lmbsol,wnsol,zetsol,ordol] = mode_migration(Afunc,Vs,opts); %poles as a function of V
Vmpsflol = fzero(@(Vmps)max(real(eigrange(Afunc(Vmps),chkrng))),Vguess); %find flutter points
lmbsfol = eig(Afunc(Vmpsflol)); %poles at flutter
lmbslo = eig(Afunc(Vlo)); %poles at V start
lmbshi = eig(Afunc(Vhi)); %poles at V end
lmbsVmps = eig(Afunc(Vmps)); %poles at model flight speed

if ~isempty(coeffsIC)
    Afunc0 = @(Vmps)Ndof6_FDFLEX(AEC6,ModeShape,FEM,Vmps,aeroProp,massProp,coeffsIC);
    [lmbsol0,wnsol0,zetsol0,ordol0] = mode_migration(Afunc0,Vs,opts); %poles as a function of V
    Vmpsflol0 = fzero(@(Vmps)max(real(eigrange(Afunc0(Vmps),chkrng))),Vguess); %flutter point
    lmbsfol0 = eig(Afunc0(Vmpsflol0)); %poles at flutter
    lmbsVmps0 = eig(Afunc0(Vmps)); %poles at model flight speed
else
    lmbsol0 = [];
    wnsol0 = [];
    zetsol0 = [];
    ordol0 = [];
    Vmpsflol0 = [];
    lmbsfol0 = [];
    lmbsVmps0 = [];
end

if ~isempty(coeffsIC)
    %plot both model and the IC
    figure;
    hmol0 = plot(real(lmbsol0.'),imag(lmbsol0.'),'LineWidth',2,'Color',0.8*[1 1 1]);
    hold on
    hmfl = plot(real(lmbsfol0),imag(lmbsfol0),'*','Color',0.5*[1 1 1],'MarkerSize',6,'LineWidth',1);
    hmol = plot(real(lmbsol.'),imag(lmbsol.'),'LineWidth',2);
    hmlo = plot(real(lmbslo(:,1)),imag(lmbslo(:,1)),'g.','MarkerSize',15);
    hmhi = plot(real(lmbshi(:,end)),imag(lmbshi(:,end)),'r.','MarkerSize',15);
    hmVm = plot(real(lmbsVmps(:,end)),imag(lmbsVmps(:,end)),'bx','MarkerSize',8,'LineWidth',2);
    hmVm0 = plot(real(lmbsVmps0(:,end)),imag(lmbsVmps0(:,end)),'x','Color',0.5*[1 1 1],'MarkerSize',6,'LineWidth',2);
    hmfl = plot(real(lmbsfol),imag(lmbsfol),'m*','MarkerSize',7,'LineWidth',1);
    axis([-30 15 0 150])
    sgrid
    legend([hmlo,hmhi,hmVm,hmfl,hmol0(1),hmVm0],['Poles at V = ' num2str(Vlo) ' m/s'],...
        ['Poles at V = ' num2str(Vhi) ' m/s'],...
        ['Identified Poles at V = ' num2str(Vmps) ' m/s'],...
        ['Flutter Velocity = ' num2str(Vmpsflol,'%5.2f') ' m/s'],...
        ['Initial Model Locus (V_f = ' num2str(Vmpsflol0,'%5.2f') ' m/s)'],...
        ['Initial Model Poles at V = ' num2str(Vmps) ' m/s'],'location','best');
else
    %plot only the model
    figure;
    hmol = plot(real(lmbsol.'),imag(lmbsol.'),'LineWidth',2);
    hold on
    hmlo = plot(real(lmbslo(:,1)),imag(lmbslo(:,1)),'g.','MarkerSize',15);
    hmhi = plot(real(lmbshi(:,end)),imag(lmbshi(:,end)),'r.','MarkerSize',15);
    hmVm = plot(real(lmbsVmps(:,end)),imag(lmbsVmps(:,end)),'bx','MarkerSize',8,'LineWidth',2);
    hmfl = plot(real(lmbsfol),imag(lmbsfol),'m*','MarkerSize',7,'LineWidth',1);
    axis([-30 15 0 150])
    
    legend([hmlo,hmhi,hmVm,hmfl],['Poles at V = ' num2str(Vlo) ' m/s'],...
        ['Poles at V = ' num2str(Vhi) ' m/s'],...
        ['Poles at V = ' num2str(Vmps) ' m/s'],...
        ['Flutter Velocity = ' num2str(Vmpsflol,'%5.2f') ' m/s'],'location','best');
end

%store outputs
outs.lmbsol = lmbsol;
outs.wnsol = wnsol;
outs.zetsol = zetsol;
outs.ordol = ordol;
outs.lmbsol0 = lmbsol0;
outs.wnsol0 = wnsol0;
outs.zetsol0 = zetsol0;
outs.ordol0 = ordol0;
outs.Vmpsflol = Vmpsflol;
outs.Vmpsflol0 = Vmpsflol0;
outs.lmbsfol = lmbsfol;
outs.lmbslo = lmbslo;
outs.lmbshi = lmbshi;
outs.lmbsVmps = lmbsVmps;
outs.lmbsVmps0 = lmbsVmps0;

s=-100:1:100;
Q=0.01 ; Z=0.01; a=0.1; k=200; p=1; c=2.1;
f=c.*(1-exp(-k.*(abs(s).^p) ) ) ;
s_dot = -Q.*(abs(s).^f).*sign(s) -Z.*(abs(s).^a).*s ;
 plot(s,s_dot)
xlabel('$ s $', 'Interpreter','latex' ,'FontSize',20)
ylabel('$\dot{s}$', 'Interpreter','latex' ,'FontSize',20)
initWorkspace


v=0.5;
w=-0.5:0.01:0.5;

K=w./v;

k=K(1);
dk=K(2)-K(1);

X_vec=zeros(1);
Y_vec=zeros(1);
counter=1;
normalCounter=1;
for kappa=-0.8:0.05:0.8
    for dkappa=-0.2:0.01:0.2
        [X,Y] = pointsOnClothoid( 0, 0, 0, kappa, dkappa,3,100 ) ;
        X_vec(counter:(counter+101))=[nan,X,nan];
        Y_vec(counter:(counter+101))=[nan,Y,nan];
        counter=counter+100;
        normalCounter=normalCounter+1;
    end
end
figure()
axis equal
hold on
plot(X_vec,Y_vec,'b')
normalCounter
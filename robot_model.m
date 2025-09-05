% Robot kolunu matrislerle tanimlama

% DH (Denavit-Hartenberg) parametreleri
% Format: [a, alpha, d, theta]
DH_params = [
    0.1, 0, 0.1, 0;  % 1. Eklem
    0.1, 0, 0, 0;    % 2. Eklem
    0.1, 0, 0, 0     % 3. Eklem
];

% Robotun eklem sayisi
num_joints = size(DH_params, 1);

% Ters Kinematik fonksiyonu
function q_ik = inverseKinematics(target_pos)
    x = target_pos(1);
    y = target_pos(2);
    z = target_pos(3);
    
    l1 = 0.1;
    l2 = 0.1;
    l3 = 0.1;
    
    q3_cos = (x^2 + y^2 + z^2 - l1^2 - l2^2 - l3^2) / (2 * l2 * l3);
    q3 = atan2(sqrt(1 - q3_cos^2), q3_cos);
    
    q2 = atan2(z, sqrt(x^2 + y^2)) - atan2(l3*sin(q3), l2 + l3*cos(q3));
    
    q1 = atan2(y, x);
    
    q_ik = [q1; q2; q3];
end

%---------------------------------------------------
% YAPAY ZEKA SIMULASYONU: NESNE AYIRMA PROJESI
%---------------------------------------------------

% Robotun ilk konumunu tanimlayan eklem acilari
q = zeros(num_joints, 1);

% Robotu gorsellestirmek icin TEK bir figure olustur
figure;
hold on;
grid on;
axis equal;
xlabel('X Axis'); % Ekseni ingilizceye çeviriyorum
ylabel('Y Axis'); % Ekseni ingilizceye çeviriyorum
zlabel('Z Axis'); % Ekseni ingilizceye çeviriyorum
title('Robot Object Sorting Animation'); % Başlığı İngilizceye çevirdim
view(3);

% Farkli nesnelerin yeni ve GUVENLI hedef pozisyonlari
target_positions = [
    0.15, 0.05, 0.05;   % 1. Hedef
    -0.15, 0.05, 0.05;  % 2. Hedef
    0, -0.15, 0.05      % 3. Hedef
];

% Her bir nesne icin dongu
for i = 1:size(target_positions, 1)
    % Hedef pozisyonu al
    current_target = target_positions(i, :);
    
    % Ters kinematik ile eklem acilarini bul
    q_new = inverseKinematics(current_target);
    
    % Robotu hedefe dogru hareket ettirme animasyonu
    steps = 50; % 50 adimda hareket etsin
    q_interp = zeros(num_joints, steps);
    for k = 1:num_joints
        q_interp(k, :) = linspace(q(k), q_new(k), steps);
    end
    
    for j = 1:steps
        % Eklem acilarini guncelle
        q = q_interp(:, j);
        
        % Cizimi her adimda yenile
        cla; % Onceki cizimi temizle
        current_transform = eye(4);
        plot3(0, 0, 0, 'o', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
        
        for k = 1:num_joints
            a = DH_params(k, 1);
            alpha = DH_params(k, 2);
            d = DH_params(k, 3);
            theta = DH_params(k, 4) + q(k);
            
            T = [
                cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
                sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                0,           sin(alpha),            cos(alpha),           d;
                0,           0,                     0,                    1
            ];
            
            new_transform = current_transform * T;
            
            link_start = current_transform(1:3, 4);
            link_end = new_transform(1:3, 4);
            plot3([link_start(1), link_end(1)], [link_start(2), link_end(2)], [link_start(3), link_end(3)], 'k', 'LineWidth', 2);
            plot3(link_end(1), link_end(2), link_end(3), 'o', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
            
            current_transform = new_transform;
        end
        
        drawnow; % Cizim penceresini guncelle
        
        % GIF olusturma kodu buraya eklenecek
        % GIF olusturma
frame = getframe(gcf);
im = frame2im(frame);
[imind, cm] = rgb2ind(im, 256);

% Dosya adi
filename = 'Robot Object Sorting Animation';

% Ilk kareyi dosyaya yaz
if i == 1 && j == 1
    imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.05);
else
    % Diger kareleri ekle
    imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
end
    end
end
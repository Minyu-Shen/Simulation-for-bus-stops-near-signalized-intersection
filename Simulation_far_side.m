clear;clc;
% rng(2);
global serving_rate;
global cs_number;
global berth_number;
global buffer_number;
global jam_spacing;
global free_speed;
global back_speed;
global moveup_speed;
global cycle_length_number;
global green_ratio;
global sim_size;
global il;

% cs_number = [0.15,0.3,0.45,0.6,0.75];
global cs;
cs = 0.6;

pl = 1; % plot handler

if cs==1
    sim_size = 300;
else
    if pl
        sim_size = 20;
    else
        sim_size = 50000;
    end
end
berth_number = 2;
buffer_number = 3:1:3;

if pl
    cycle_length_number = 140:1:140;
else
    cycle_length_number = 80:1:240;
end

green_ratio = 0.7;
jam_spacing = 12;
free_speed = 20 / 3.6;
back_speed = 25 / 3.6;
moveup_speed = 20 / 3.6;
serving_rate = 1/25;
il = 3;
mh = jam_spacing / moveup_speed;
bh = jam_spacing / back_speed;
ih = il*jam_spacing/moveup_speed;

% mh=0;
% bh=0;
% fh=0;

%% simulation procedures
cs_size = sum(cs_number~=-1);
c_size = sum(berth_number~=-1);
d_size = sum(buffer_number~=-1);
cl_size = sum(cycle_length_number~=-1);

for l=1:cl_size
    cycle_length = cycle_length_number(l);
    green_time = cycle_length * green_ratio;
    if cs==0
        serv_time(1 : sim_size) = unifrnd(1/serving_rate, 1/serving_rate);
    else
        serv_time(1 : sim_size) = gamrnd(1/cs^2, cs^2/serving_rate, 1, sim_size);
    end
    
    for k=1:c_size
        c = berth_number(k);
        for j=1:d_size
            u = zeros(sim_size, 1);
            d = buffer_number(j);
            d0 = mod(d,c);
            n = floor(d/c);
            
            dq_m = zeros(sim_size, 1);
            bff_pos = zeros(sim_size, 1);
            bth_pos = zeros(sim_size, 1);
            bff_times = zeros(sim_size, 1);
            arr_bff_m = zeros(sim_size, n+1);
            lv_bff_m = zeros(sim_size, n+1);
            end_serv_m = zeros(sim_size, 1);
            wait_bth = zeros(sim_size, 1);
            lv_bth_m = zeros(sim_size, 1);
            
            for i=1:sim_size
                if i == 1
                    % starting from green period
                    dq_m(i) = 0;
                    bff_pos(i) = 0;
                    bth_pos(i) = 1;
                    bff_times(i) = 0;
                    end_serv_m(i) = dq_m(i) + (c+d)*mh + ih + serv_time(i);
                    wait_bth(i) = 0;
                    lv_bth_m(i) = end_serv_m(i);
                    
                else % i > 1 starts
                    %% buffer_pos(i-1) == 0 starts
                    if bff_pos(i-1) == 0
                        temp_dq_m = dq_m(i-1) + mh +bh;
                        if bth_pos(i-1) == c
                            if d==0
                                bth_pos(i) = 1;
                                bff_pos(i) = 0;
                                bff_times(i) = 0;
                                if mod(lv_bth_m(i-1) + bh, cycle_length) <= green_time
                                    dq_m(i) = lv_bth_m(i-1) + bh;
                                else
                                    dq_m(i) = cycle_length - mod(lv_bth_m(i-1), cycle_length) + lv_bth_m(i-1) + bh;
                                end
                                end_serv_m(i) = dq_m(i) + (c+d)*mh + ih + serv_time(i);
                                wait_bth(i) = 0;
                                lv_bth_m(i) = end_serv_m(i);
                            else % last berth is c and d~=0
                                if mod(temp_dq_m, cycle_length) <= green_time
                                    dq_m(i) = temp_dq_m;
                                    bff_pos(i) = 1;
                                    bth_pos(i) = det_bth(1, c);
                                    bff_times(i) = 1;
                                    lv_bff_m(i, 1) = lv_bth_m(i-1)+bh;
                                    end_serv_m(i) = lv_bff_m(i, 1) + c*mh + serv_time(i);
                                    wait_bth(i) = 0;
                                    lv_bth_m(i) = end_serv_m(i);
                                else % meet the red period, wait until the greee period
                                    nxt_g = cycle_length - mod(dq_m(i-1), cycle_length) + dq_m(i-1) + bh;
                                    if nxt_g < lv_bth_m(i-1) + bh
                                        dq_m(i) = lv_bth_m(i-1) + bh;
                                        bff_pos(i) = 1;
                                        bth_pos(i) = det_bth(1, c);
                                        bff_times(i) = 1;
                                        lv_bff_m(i, 1) = lv_bth_m(i-1)+bh;
                                        end_serv_m(i) = lv_bff_m(i, 1) + c*mh + serv_time(i);
                                        wait_bth(i) = 0;
                                        lv_bth_m(i) = end_serv_m(i);
                                    else
                                        dq_m(i) = nxt_g;
                                        bff_pos(i) = 0;
                                        bth_pos(i) = 1;
                                        bff_times(i) = 0;
                                        end_serv_m(i) = dq_m(i) + (c+d)*mh + ih + serv_time(i);
                                        wait_bth(i) = 0;
                                        lv_bth_m(i) = end_serv_m(i);
                                    end
                                end
                            end
                        else % last bus's berth < c
                            bff_pos(i) = 0;
                            bff_times(i) = 0;
                            if mod(temp_dq_m, cycle_length) <= green_time
                                dq_m(i) = temp_dq_m;
                                bth_pos(i) = bth_pos(i-1) + 1;
                                end_serv_m(i) = dq_m(i) + (c+d-bth_pos(i)+1)*mh + ih + serv_time(i);
                                wait_bth(i) = max(0, lv_bth_m(i-1) + bh - end_serv_m(i));
                                lv_bth_m(i) = end_serv_m(i) + wait_bth(i);
                            else
                                nxt_g = cycle_length - mod(temp_dq_m, cycle_length) + temp_dq_m + bh;
                                dq_m(i) = nxt_g;
                                if nxt_g < lv_bth_m(i-1) + bh
                                    dq_m(i) = lv_bth_m(i-1) + bh;
                                    bth_pos(i) = bth_pos(i-1)+1;
                                    end_serv_m(i) = dq_m(i) + (d+c-bth_pos(i)+1+il)*mh + serv_time(i);
                                    wait_bth(i) = 0;
                                    lv_bth_m(i) = end_serv_m(i);
                                else
                                    dq_m(i) = nxt_g;
                                    bth_pos(i) = 1;
                                    bff_times(i) = 0;
                                    end_serv_m(i) = dq_m(i) + (c+d)*mh + ih + serv_time(i);
                                    wait_bth(i) = 0;
                                    lv_bth_m(i) = end_serv_m(i);
                                end
                            end
                        end
                        
                        %% buffer_pos(i-1) == 0 ends
                    elseif bff_pos(i-1) < d
                        %% 0< buffer_pos(i-1) < d
                        temp_dq_m = dq_m(i-1) + mh + bh;
                        % determine berth and buffer
                        if mod(temp_dq_m, cycle_length) <= green_time
                            dq_m(i) = temp_dq_m;
                            bff_pos(i) = bff_pos(i-1) + 1;
                            bth_pos(i) = det_bth(bff_pos(i), c);
                        else
                            dq_m(i) = dq_m(i-1) + cycle_length - mod(dq_m(i-1), cycle_length) + bh;
                            flag = 0;
                            for p=1:1:bff_times(i-1)
                                evr_bff_pos = bff_pos(i-1) - (p-1)*c;
                                tp = dq_m(i) + (d-evr_bff_pos+1)*mh + ih;
                                if tp < lv_bff_m(i-1, p) + bh
                                    bff_pos(i) = evr_bff_pos + 1;
                                    bth_pos(i) = det_bth(bff_pos(i), c);
                                    flag = 1;
                                    break;
                                end
                            end
                            if flag == 0 % judge berth point
                                tp = dq_m(i) + ih + (d+c-bth_pos(i-1))*mh;
                                if tp < lv_bth_m(i-1) + bh
                                    if bth_pos(i-1) == c
                                        bff_pos(i) = 1;
                                        bth_pos(i) = 1;
                                    else
                                        bff_pos(i) = 0;
                                        bth_pos(i) = bth_pos(i-1) + 1;
                                    end
                                else
                                    bff_pos(i) = 0;
                                    bth_pos(i) = 1;
                                end
                            end
                        end
                        
                        % determine lv_bff_m
                        
                        bff_times(i) = ceil(bff_pos(i)/c);
                        if bff_times(i) == 0 % lv_bff is not important !!!
                            end_serv_m(i) = dq_m(i) + (c+d-bth_pos(i)+1+il)*mh + serv_time(i);
                            wait_bth(i) = max(0, lv_bth_m(i-1) + bh - end_serv_m(i));
                            lv_bth_m(i) = end_serv_m(i) + wait_bth(i);
                        else % wait at buffer before service
                            
                            if bff_times(i) == bff_times(i-1) % same convoy
                                if (mod(bff_pos(i), c) == 1 && c~=1) || (mod(bff_pos(i), c) == 0 && c==1) % the first bus
                                    for p=1:1:bff_times(i)-1
                                        lv_bff_m(i, p) = lv_bff_m(i-1, p+1) + bh;
                                    end
                                    lv_bff_m(i, bff_times(i)) = lv_bth_m(i-1) + bh;
                                else
                                    for p=1:1:bff_times(i)
                                        lv_bff_m(i, p) = lv_bff_m(i-1, p) + bh;
                                    end
                                end
                                end_serv_m(i) = lv_bff_m(i, bff_times(i)) + c*mh + serv_time(i);
                                
                            elseif bff_times(i) > bff_times(i-1)
                                for p=1:1:bff_times(i-1)
                                    lv_bff_m(i, p) = lv_bff_m(i-1, p)+bh;
                                end
                                lv_bff_m(i, bff_times(i)) = lv_bth_m(i-1) + bh;
                                end_serv_m(i) = lv_bff_m(i, bff_times(i)) + c*mh + serv_time(i);
                            else % bff_times(i) < bff_times(i-1)
                                if (mod(bff_pos(i), c) == 1 && c~=1) || (mod(bff_pos(i), c) == 0 && c==1)
                                    lv_bff_m(i, bff_times(i)) = lv_bth_m(i-1) + bh;
                                    for p=bff_times(i)-1:-1:1
                                        lv_bff_m(i, p) = lv_bff_m(i-1, bff_times(i-1)-bff_times(i)+p) + bh;
                                    end
                                    end_serv_m(i) = lv_bff_m(i, bff_times(i)) + c*mh + serv_time(i);
                                else
                                    for p=bff_times(i):-1:1
                                        lv_bff_m(i, p) = lv_bff_m(i-1, bff_times(i-1)-bff_times(i)+p) + bh;
                                    end
                                    end_serv_m(i) = lv_bff_m(i, bff_times(i)) + c*mh + serv_time(i);
                                end
                            end
                            wait_bth(i) = max(0, lv_bth_m(i-1) + bh - end_serv_m(i));
                            lv_bth_m(i) = end_serv_m(i) + wait_bth(i);
                        end
                        
                        %% 0 < buffer_pos(i-1) < d ends
                    else
                        %% buffer_pos(i-1) == d starts
                        temp_dq_m = lv_bff_m(i-1, 1) + bh;
                        
                        % determine berth and buffer
                        if mod(temp_dq_m, cycle_length) <= green_time
                            dq_m(i) = temp_dq_m;
                            flag=0;
                            for p=1:1:bff_times(i-1)
                                evr_bff_pos = d - (p-1)*c;
                                tp = dq_m(i) + (d-evr_bff_pos+1)*mh + ih;
                                if tp < lv_bff_m(i-1, p) + bh
                                    if p==1
                                        bff_pos(i) = d-c+1;
                                    else
                                        bff_pos(i) = evr_bff_pos + 1;
                                    end
                                    bth_pos(i) = det_bth(bff_pos(i), c);
                                    flag = 1;
                                    break;
                                end
                            end
                            if flag == 0 % judge berth point
                                tp = dq_m(i) + ih + (d+c-bth_pos(i-1))*mh;
                                if tp < lv_bth_m(i-1) + bh
                                    if bth_pos(i-1) == c
                                        bff_pos(i) = 1;
                                        bth_pos(i) = 1;
                                    else
                                        bff_pos(i) = 0;
                                        bth_pos(i) = bth_pos(i-1) + 1;
                                    end
                                else
                                    bff_pos(i) = 0;
                                    bth_pos(i) = 1;
                                end
                            end
                            
                            % bff_pos(i) = d-c+1;
                            % bth_pos(i) = det_bth(bff_pos(i), c);
                        else
                            dq_m(i) = temp_dq_m + cycle_length - mod(temp_dq_m, cycle_length) + bh;
                            flag = 0;
                            for p=1:1:bff_times(i-1)
                                evr_bff_pos = d - (p-1)*c;
                                tp = dq_m(i) + (d-evr_bff_pos+1)*mh + ih;
                                if tp < lv_bff_m(i-1, p) + bh
                                    if p==1
                                        bff_pos(i) = d-c+1;
                                    else
                                        bff_pos(i) = evr_bff_pos + 1;
                                    end
                                    bth_pos(i) = det_bth(bff_pos(i), c);
                                    flag = 1;
                                    break;
                                end
                            end
                            if flag == 0 % judge berth point
                                tp = dq_m(i) + ih + (d+c-bth_pos(i-1))*mh;
                                if tp < lv_bth_m(i-1) + bh
                                    if bth_pos(i-1) == c
                                        bff_pos(i) = 1;
                                        bth_pos(i) = 1;
                                    else
                                        bff_pos(i) = 0;
                                        bth_pos(i) = bth_pos(i-1) + 1;
                                    end
                                else
                                    bff_pos(i) = 0;
                                    bth_pos(i) = 1;
                                end
                            end
                        end
                        
                        % determine lv_bff_m
                        bff_times(i) = ceil(bff_pos(i)/c);
                        if bff_times(i) == 0 % lv_bff is not important !!!
                            end_serv_m(i) = dq_m(i) + (c+d-bth_pos(i)+1+il)*mh + serv_time(i);
                            wait_bth(i) = max(0, lv_bth_m(i-1) + bh - end_serv_m(i));
                            lv_bth_m(i) = end_serv_m(i) + wait_bth(i);
                        else % wait at buffer before service
                            
                            if bff_times(i) == bff_times(i-1) % same convoy
                                if (mod(bff_pos(i), c) == 1 && c~=1) || (c==1) % the first bus
                                    for p=1:1:bff_times(i)-1
                                        lv_bff_m(i, p) = lv_bff_m(i-1, p+1);
                                    end
                                    lv_bff_m(i, bff_times(i)) = lv_bth_m(i-1) + bh;
                                else
                                    for p=1:1:bff_times(i)
                                        lv_bff_m(i, p) = lv_bff_m(i-1, p) + bh;
                                    end
                                end
                                end_serv_m(i) = lv_bff_m(i, bff_times(i)) + c*mh + serv_time(i);
                                
                            elseif bff_times(i) > bff_times(i-1)
                                for p=1:1:bff_times(i-1)
                                    lv_bff_m(i, p) = lv_bff_m(i-1, p)+bh;
                                end
                                lv_bff_m(i, bff_times(i)) = lv_bth_m(i-1) + bh;
                                end_serv_m(i) = lv_bff_m(i, bff_times(i)) + c*mh + serv_time(i);
                            else % bff_times(i) < bff_times(i-1)
                                if (mod(bff_pos(i), c) == 1 && c~=1) || (c==1)
                                    lv_bff_m(i, bff_times(i)) = lv_bth_m(i-1) + bh;
                                    for p=bff_times(i)-1:-1:1
                                        lv_bff_m(i, p) = lv_bff_m(i-1, bff_times(i-1)-bff_times(i)+p+1) + bh;
                                    end
                                    end_serv_m(i) = lv_bff_m(i, bff_times(i)) + c*mh + serv_time(i);
                                else
                                    for p=bff_times(i):-1:1
                                        lv_bff_m(i, p) = lv_bff_m(i-1, bff_times(i-1)-bff_times(i)+p) + bh;
                                    end
                                    end_serv_m(i) = lv_bff_m(i, bff_times(i)) + c*mh + serv_time(i);
                                end
                            end
                            wait_bth(i) = max(0, lv_bth_m(i-1) + bh - end_serv_m(i));
                            lv_bth_m(i) = end_serv_m(i) + wait_bth(i);
                        end
                        
                    end
                    %% buffer_pos(i-1) == d ends
                    
                end
            end
            end_time = lv_bth_m(sim_size);
            capacity(l,j) = 3600 * sim_size / end_time;
        end
    end
end

if ~pl
    plot(cycle_length_number,capacity,'linewidth',2,'linestyle','-','color',[111/255, 122/255, 117/255]);
    hold on;
end

if pl
    plot_number = sim_size;
    % plot stop line
    t = [0, lv_bth_m(plot_number)];
    y = [0, 0];
    plot(t, y, '-', 'Color', 'k', 'LineWidth', 3);
    hold on;
    
    %plot signal
    for i=1:ceil(lv_bth_m(plot_number)/cycle_length)
        signal_y = [il*jam_spacing, il*jam_spacing];
        signal_red_x = [(i-1)*cycle_length,(i-1)*cycle_length+green_time];
        signal_green_x = [(i-1)*cycle_length+green_time,i*cycle_length];
        line(signal_red_x,signal_y,'Color','g','LineWidth',4,'LineStyle','-');
        line(signal_green_x,signal_y,'Color','r','LineWidth',4,'LineStyle','-');
    end
    
    for i=1:c+d
        %plot berth and buffer
        x = [0,lv_bth_m(plot_number)];
        y = [(il+i)*jam_spacing, (il+i)*jam_spacing];
        plot(x,y,'--');
        hold on;
    end
    
    for i=1:plot_number
        %1
        dq_x = []; dq_y=[];
        if bff_pos(i)==0
            dq_x = [dq_m(i), dq_m(i)+(d+c-bth_pos(i)+1+il)*mh];
            dq_y = [0,(d+c-bth_pos(i)+1+il)*jam_spacing];
        else
            dq_x = [dq_m(i), dq_m(i)+(d-bff_pos(i)+1+il)*mh];
            dq_y = [0, (d-bff_pos(i)+1+il)*jam_spacing];
        end
        line(dq_x, dq_y, 'linewidth',2);
        
        % 2 plot waiting at buffer
        for p=1:1:bff_times(i)
            temp_bff_pos = bff_pos(i) - (p-1)*c;
            if p==1
                bff_x = [dq_x(2), lv_bff_m(i, p)];
                bff_y = [dq_y(2), (d-temp_bff_pos+1+il)*jam_spacing];
                line(bff_x,bff_y,'linewidth',3);
            else
                bff_x = [lv_bff_m(i, p-1)+c*mh, lv_bff_m(i, p)];
                bff_y = [(d-temp_bff_pos+1+il)*jam_spacing, (d-temp_bff_pos+1+il)*jam_spacing];
                line(bff_x, bff_y,'linewidth',3);
            end
        end
        
        % 3 plot moving behavior in the buffers
        for p=2:1:bff_times(i)
            mv_bff_x = [lv_bff_m(i, p-1), lv_bff_m(i, p-1)+c*mh];
            mv_bff_y = [(d-(bff_pos(i) - (p-2)*c)+1+il)*jam_spacing, (d-(bff_pos(i) - (p-1)*c)+1+il)*jam_spacing];
            line(mv_bff_x, mv_bff_y, 'linewidth', 1.5);
        end
        
        % 4 plot moving behavior to berths
        if bff_times(i) ~= 0
            mv_bth_x = [lv_bff_m(i, bff_times(i)), lv_bff_m(i, bff_times(i))+c*mh];
            mv_bth_y = [(d-(bff_pos(i) - (bff_times(i)-1)*c)+1+il)*jam_spacing, (d+c-bth_pos(i)+1+il)*jam_spacing];
            line(mv_bth_x, mv_bth_y, 'linewidth', 1.5);
        end
        
        % 5 plot service time
        if bff_times(i)==0
            arr_bth_t = dq_m(i)+(d+c-bth_pos(i)+1+il)*mh;
        else
            arr_bth_t = lv_bff_m(i, bff_times(i))+c*mh;
        end
        serv_x = [arr_bth_t, arr_bth_t+serv_time(i)];
        serv_y = [(d+c-bth_pos(i)+1+il)*jam_spacing, (d+c-bth_pos(i)+1+il)*jam_spacing];
        line(serv_x, serv_y, 'linewidth', 3, 'color', 'k');
        
        % 6 plot waiting in berth and departing
        fake=5;
        if wait_bth(i) == 0
            dpt_x = [arr_bth_t+serv_time(i), arr_bth_t+serv_time(i)+fake];
            dpt_y = [(d+c-bth_pos(i)+1+il)*jam_spacing, (d+c-bth_pos(i)+1+il)*jam_spacing+fake*5];
            line(dpt_x, dpt_y, 'linewidth', 2.5);
        else
            wb_x = [arr_bth_t+serv_time(i), arr_bth_t+serv_time(i)+wait_bth(i)];
            wb_y = [(d+c-bth_pos(i)+1+il)*jam_spacing, (d+c-bth_pos(i)+1+il)*jam_spacing];
            line(wb_x, wb_y, 'linewidth', 1.5);
            dpt_x = [arr_bth_t+serv_time(i)+wait_bth(i), arr_bth_t+serv_time(i)+wait_bth(i)+fake];
            dpt_y = [(d+c-bth_pos(i)+1+il)*jam_spacing, (d+c-bth_pos(i)+1+il)*jam_spacing+fake*5];
            line(dpt_x, dpt_y, 'linewidth', 2.5);
        end
    end
end

function result = det_bth(bff_pos, c)

if mod(bff_pos, c) == 0
    result = c;
else
    result = mod(bff_pos, c);
end

end

function varargout = markArteriesandVeins(varargin)
% MARKARTERIESANDVEINS MATLAB code for markArteriesandVeins.fig
%      MARKARTERIESANDVEINS, by itself, creates a new MARKARTERIESANDVEINS or raises the existing
%      singleton*.
%
%      H = MARKARTERIESANDVEINS returns the handle to a new MARKARTERIESANDVEINS or the handle to
%      the existing singleton*.
%
%      MARKARTERIESANDVEINS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MARKARTERIESANDVEINS.M with the given input arguments.
%
%      MARKARTERIESANDVEINS('Property','Value',...) creates a new MARKARTERIESANDVEINS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before markArteriesandVeins_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to markArteriesandVeins_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help markArteriesandVeins

% Last Modified by GUIDE v2.5 28-May-2019 12:04:47

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @markArteriesandVeins_OpeningFcn, ...
                   'gui_OutputFcn',  @markArteriesandVeins_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before markArteriesandVeins is made visible.
function markArteriesandVeins_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to markArteriesandVeins (see VARARGIN)

% Choose default command line output for markArteriesandVeins
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes markArteriesandVeins wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = markArteriesandVeins_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --------------------------------------------------------------------
function Menu_Callback(hObject, eventdata, handles)
% hObject    handle to Menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function menuLoadData_Callback(hObject, eventdata, handles)
% hObject    handle to menuLoadData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global mAR

[filename,pathname] = uigetfile({'*.mat;*.tiff;*.tif'},'Please select the Angiogram Data');
h = waitbar(0,'Please wait... loading the data');
[~,~,ext] = fileparts(filename);

if strcmp(ext,'.mat')
    temp = load([pathname filename]);
    fn = fieldnames(temp);
    mAR.angio = temp.(fn{1});
elseif  strcmp(ext,'.tiff') || strcmp(ext,'.tif')
    info = imfinfo([pathname filename]);
    for u = 1:length(info)
        if u == 1
            temp = imread([pathname filename],1);
            angio = zeros([length(info) size(temp)]);
            angio(u,:,:) = temp;
        else
            angio(u,:,:) = imread([pathname filename],u);
        end
    end
    mAR.angio = angio;
end

[z,x,y] = size(mAR.angio);

set(handles.edit_zStartFrame,'String',num2str(1));
set(handles.edit_zEndFrame,'String',num2str(z));

set(handles.edit_xStartFrame,'String',num2str(1));
set(handles.edit_xEndFrame,'String',num2str(x));

set(handles.edit_yStartFrame,'String',num2str(1));
set(handles.edit_yEndFrame,'String',num2str(y));

close(h)

draw(hObject, eventdata, handles)

function edit_zStartFrame_Callback(hObject, eventdata, handles)
% hObject    handle to edit_zStartFrame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_zStartFrame as text
%        str2double(get(hObject,'String')) returns contents of edit_zStartFrame as a double

draw(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function edit_zStartFrame_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_zStartFrame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_zEndFrame_Callback(hObject, eventdata, handles)
% hObject    handle to edit_zEndFrame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_zEndFrame as text
%        str2double(get(hObject,'String')) returns contents of edit_zEndFrame as a double

draw(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function edit_zEndFrame_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_zEndFrame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_xStartFrame_Callback(hObject, eventdata, handles)
% hObject    handle to edit_xStartFrame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_xStartFrame as text
%        str2double(get(hObject,'String')) returns contents of edit_xStartFrame as a double

draw(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function edit_xStartFrame_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_xStartFrame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_xEndFrame_Callback(hObject, eventdata, handles)
% hObject    handle to edit_xEndFrame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_xEndFrame as text
%        str2double(get(hObject,'String')) returns contents of edit_xEndFrame as a double

draw(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function edit_xEndFrame_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_xEndFrame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_yStartFrame_Callback(hObject, eventdata, handles)
% hObject    handle to edit_yStartFrame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_yStartFrame as text
%        str2double(get(hObject,'String')) returns contents of edit_yStartFrame as a double

draw(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function edit_yStartFrame_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_yStartFrame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_yEndFrame_Callback(hObject, eventdata, handles)
% hObject    handle to edit_yEndFrame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_yEndFrame as text
%        str2double(get(hObject,'String')) returns contents of edit_yEndFrame as a double

draw(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function edit_yEndFrame_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_yEndFrame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --------------------------------------------------------------------
function loadGraphData_Callback(hObject, eventdata, handles)
% hObject    handle to loadGraphData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global mAR

[filename,pathname] = uigetfile('*.mat','Please select the Angiogram Data');
load([pathname filename]);
mAR.Graph = Graph;


function draw(hObject, eventdata, handles)

global mAR


[Sz,Sy,Sx] = size(mAR.angio);


%%%% Read display range
Zstartframe = min(max(str2double(get(handles.edit_zStartFrame,'String')),1),Sz);
Zendframe = min(max(str2double(get(handles.edit_zEndFrame,'String')),1),Sz);

Xstartframe = min(max(str2double(get(handles.edit_xStartFrame,'String')),1),Sx);
Xendframe = min(max(str2double(get(handles.edit_xEndFrame,'String')),1),Sx);

Ystartframe = min(max(str2double(get(handles.edit_yStartFrame,'String')),1),Sy);
Yendframe = min(max(str2double(get(handles.edit_yEndFrame,'String')),1),Sy);

img = squeeze(max(mAR.angio(Zstartframe:Zendframe,:,:),[],1));

axes(handles.axes1)

I_h = imagesc(img);
colormap('gray')
xlim([Xstartframe Xendframe]);
ylim([Ystartframe Yendframe]);

axis image;
axis on

if get(handles.radiobutton_displayGraph,'Value')
    
    nodes = mAR.Graph.nodes;
    edges = mAR.Graph.edges;
            
    lst = find(nodes(:,1)>=Xstartframe & nodes(:,1)<=Xendframe & ...
        nodes(:,2)>=Ystartframe & nodes(:,2)<=Yendframe & ...
        nodes(:,3)>=Zstartframe & nodes(:,3)<=Zendframe );
    
    hold on
    
    plot(mAR.Graph.nodes(lst,1),mAR.Graph.nodes(lst,2),'g.');
    
    if isfield(mAR.Graph,'nodeType')
        lstArtery = find(nodes(:,1)>=Xstartframe & nodes(:,1)<=Xendframe & ...
            nodes(:,2)>=Ystartframe & nodes(:,2)<=Yendframe & ...
            nodes(:,3)>=Zstartframe & nodes(:,3)<=Zendframe & ...
            mAR.Graph.nodeType == 1);
        
        plot(mAR.Graph.nodes(lstArtery,1),mAR.Graph.nodes(lstArtery,2),'r.');
        
        lstVein = find(nodes(:,1)>=Xstartframe & nodes(:,1)<=Xendframe & ...
            nodes(:,2)>=Ystartframe & nodes(:,2)<=Yendframe & ...
            nodes(:,3)>=Zstartframe & nodes(:,3)<=Zendframe & ...
            mAR.Graph.nodeType == 3);
        
        plot(mAR.Graph.nodes(lstVein,1),mAR.Graph.nodes(lstVein,2),'b.');
    end
    
%     if 0
%         foo = ismember(edges,lst);
%         lst2 = find(sum(foo,2)==2);
%     
%         plot([nodes(edges(lst2,1),1) nodes(edges(lst2,2),1)]', ...
%              [nodes(edges(lst2,1),2) nodes(edges(lst2,2),2)]', 'm-' );
%     else
%         lst2 = lst( find(nB(lst)>2) );
%         plot(mAR.Graph.nodes(lst2,1),mAR.Graph.nodes(lst2,2),'mo');
%     end
    hold off
end

axis off

set(I_h, 'ButtonDownFcn', {@axes_ButtonDown, handles});


% --- Executes on button press in radiobutton_displayGraph.
function radiobutton_displayGraph_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton_displayGraph (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton_displayGraph

draw(hObject, eventdata, handles)




function axes_ButtonDown(hObject, eventdata, handles) 

global mAR

parent = (get(hObject, 'Parent'));
pts = get(parent, 'CurrentPoint');
y = pts(1,1);
x = pts(1,2);
[Sz,Sx,Sy] = size(mAR.angio);

Zstartframe = min(max(str2double(get(handles.edit_zStartFrame,'String')),1),Sz);
Zendframe = min(max(str2double(get(handles.edit_zEndFrame,'String')),1),Sz);

% find the indices close to the selected point
s = 10;
idx = find(mAR.Graph.nodes(:,2) >= x-s & mAR.Graph.nodes(:,2) <= x+s ...
    & mAR.Graph.nodes(:,1) >= y-s & mAR.Graph.nodes(:,1) <= y+s ...
    & mAR.Graph.nodes(:,3)>= Zstartframe & mAR.Graph.nodes(:,3) <= Zendframe);

% find the closet point
min_idx = 1;
for u = 1:length(idx)
    idx_x = mAR.Graph.nodes(idx(u),2);
    idx_y = mAR.Graph.nodes(idx(u),1);
    if u == 1
        min_dist = sqrt((idx_x-x)^2+(idx_y-y)^2);
    else
        dist = sqrt((idx_x-x)^2+(idx_y-y)^2);
        if dist < min_dist
            min_dist = dist;
            min_idx = u;
        end
    end
end

seg_node = idx(min_idx);
% idx_seg = find(Data.Graph.segInfo.segEndNodes(:,1) == seg_node | Data.Graph.segInfo.segEndNodes(:,2) == seg_node);
selected_segment = mAR.Graph.segInfo.nodeSegN(seg_node);

if ~isfield(mAR.Graph.segInfo,'segType')
    mAR.Graph.segInfo.segType = 2*ones(size(mAR.Graph.segInfo.segLen));    
end


if ~isfield(mAR.Graph,'nodeType')
    mAR.Graph.nodeType = 2*ones(size(mAR.Graph.nodes,1),1);    
end

if ~isfield(mAR.Graph,'edgeType')
    mAR.Graph.edgeType = 2*ones(size(mAR.Graph.edges,1),1);    
end

seg_nodes = find(mAR.Graph.segInfo.nodeSegN == selected_segment);
seg_edges = find(mAR.Graph.segInfo.edgeSegN == selected_segment);
if get(handles.radiobutton_markArtery,'value')
    if mAR.Graph.segInfo.segType(selected_segment) ~= 1
        mAR.Graph.segInfo.segType(selected_segment) = 1;
        mAR.Graph.nodeType(seg_nodes) = 1;
        mAR.Graph.edgeType(seg_edges) = 1;
    else
        mAR.Graph.segInfo.segType(selected_segment) = 2;
        mAR.Graph.nodeType(seg_nodes) = 2;
        mAR.Graph.edgeType(seg_edges) = 2;
    end
elseif get(handles.radiobutton_markVein,'value')
    if mAR.Graph.segInfo.segType(selected_segment) ~= 3
        mAR.Graph.segInfo.segType(selected_segment) = 3;
        mAR.Graph.nodeType(seg_nodes) = 3;
        mAR.Graph.edgeType(seg_edges) = 3;
    else
        mAR.Graph.segInfo.segType(selected_segment) = 2;
        mAR.Graph.nodeType(seg_nodes) = 2;
        mAR.Graph.edgeType(seg_edges) = 2;
    end
end

draw(hObject, eventdata, handles)
    


% --------------------------------------------------------------------
function menu_getSegInfo_Callback(hObject, eventdata, handles)
% hObject    handle to menu_getSegInfo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global mAR

mAR.Graph.segInfo = nodeGrps_vesSegment(mAR.Graph.nodes,mAR.Graph.edges);


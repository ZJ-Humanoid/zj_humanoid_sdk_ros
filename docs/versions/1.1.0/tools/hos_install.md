# hos 服务安装指南

## 1. 安装包使用流程简介

### 流程简介：

1. 用户在 HOS 内部指定版本并下发打包指令后，系统会自动根据该版本信息匹配所有服务版本，组合生成安装包。
2. 打包完成后，系统会将安装包上传至云端路径。
3. 在对应的个人版或企业版设备上，通过浏览器登录云端下载安装包并在本地执行安装命令，即可完成安装。

### 安装包格式说明：

**安装包命名格式如下：**

```bash
【版本类型】-install-【打包类型】-【打包时间】-【版本号】.tar.gz
```

示例：

```bash
personal-install-all-20251112-v0.0.3.tar.gz
```

参数说明：

| **分类**           | **参数名**             | **含义**     | **适用版本** |
| ------------------ | ---------------------- | ------------ | ------------ |
| **版本类型**       | personal               | 个人版安装包 | 个人版       |
| cluster            | 集群版安装包           | 企业版       |              |
| **打包类型**       | all                    | 全量打包     | 通用         |
| arm                | 大脑服务打包           | 通用         |              |
| hos                | 云上业务服务打包       | 通用         |              |
| requires           | 大脑依赖的三方服务打包 | 通用         |              |
| **企业版专属类型** | vmmanager              | 虚拟机安装包 | 企业版       |
| sdk                | SDK 相关安装包         | 企业版       |              |



**云端存储路径说明：**

云端路径：`http://112.124.10.230:30099/browser`

打包完成后，系统会将安装包上传至云端路径：

```bash
【版本类型】-【打包类型】/【公司名称（默认 naviai）】/【安装包名称】
```

示例：

```bash
cluster-arm/naviai/cluster-install-arm-20251112-v0.0.3.tar.gz
```





## 2. 安装包架构介绍

![HOS_CLAUSE](../images/HOS_clause.png)

无论是 **个人版** 还是 **集群版**，我们的安装包都是通过 **一台控制机器** 来批量管理和配置所有需要部署服务的服务器。

控制机器上运行着安装程序（基于 Ansible 工具），可以一键把服务部署到 **本机和其他机器** 上。



### 2.2 机器角色说明

#### 控制机 （需部署 Ansible + **OpenSSH** ,云端 utils目录下有对应安装）

- 单独找一台新机器
- 直接使用集群的master节点当做控制机
- 直接使用个人版运行hos服务的机器当中控制机

#### 被控机器 (需部署 **OpenSSH** )

- 需要与hos联动的大脑服务器（orin）
- 集群版node节点

#### 控制原理

- 控制机器（使用Ansible ） → 通过 **OpenSSH** → 业务服务器



### 2.3 机器工具的安装

- Ansible的安装 (控制机安装)

  ```bash
  # 解压 utils/ansible.tar.gz 
  tar -zxvf ansible.tar.gz 
  cd ansible
  
  # 离线安装ansible
  bash install_ansible.sh
  ```

- OpenSSH 的安装 （控制机和被控机器都需要安装）

  ```bash
  # 解压 utils/openssh.tar.gz 
  tar -zxvf openssh.tar.gz 
  cd openssh
  
  # 离线安装openssh
  bash install_openssh_offline.sh
  ```



## 3. hos个人版服务安装步骤

### 3.1 安装hos 个人版服务 

> **使用场景：当目标机器尚未安装过 HOS 时，可用于在全新环境中部署个人版 HOS。**

#### 操作步骤

- 解压 **个人版全量** 压缩包

  ```bash
  tar -zxvf personal-install-all-20251112-v0.0.3.tar.gz
  cd cluster_install
  ```

- 配置需要安装hos服务机器的账户密码 `nano personal.ini`

  ```ini
  # hosts.ini
  [all]
  personal-104 ansible_host=192.168.8.104 ansible_user=root ansible_ssh_pass="supcon@2025" 
  
  # 全局变量
  [all:vars]
  ansible_python_interpreter=/usr/bin/python3
  
  # 填写需要部署hos个人版服务器ip、账户、密码，ansible_ssh_pass 与 ansible_become_pass 需要保持一致
  ```
  
- 执行安装个人版服务

  ```bash
   ansible-playbook -i personal.ini personal.yaml
  ```

- 等待脚本执行完毕即可安装hos个人版服务



### 3.2 更新hos 个人版服务 

> **使用场景：当目标机器已经安装过 HOS 时，可用于更新个人版 HOS**

#### 操作步骤

- 解压 **个人版更新** 压缩包

  ```bash
  tar -zxvf personal-install-hos-20251112-v0.0.3.tar.gz
  cd cluster_install
  ```

- 配置需要安装hos服务机器的账户密码 `nano personal.ini`

  ```ini
  # hosts.ini
  [all]
  personal-104 ansible_host=192.168.8.104 ansible_user=root ansible_ssh_pass="supcon@2025" 
  
  # 全局变量
  [all:vars]
  ansible_python_interpreter=/usr/bin/python3
  
  # 填写需要部署hos个人版服务器ip、账户、密码，ansible_ssh_pass 与 ansible_become_pass 需要保持一致
  ```

- 执行安装个人版服务

  ```bash
   ansible-playbook -i personal.ini personal.yaml
  ```

- 等待脚本执行完毕即可更新hos个人版服务

  

### 3.3 hos 个人版大脑服务安装 

> **使用场景：当目标机器已经安装过个人版的 HOS 时，可用于安装或更新个人版 HOS对应的大脑服务**
>
> **注意：大脑服务的安装与更新的步骤一致**

#### 操作步骤

- 解压 **个人版大脑**压缩包

  ```bash
  tar -zxvf personal-install-arm-20251112-v0.0.3.tar.gz
  cd cluster_install
  ```

- 配置需要安装hos服务的大脑服务器的账户密码 `nano orin.ini`

  - 修改k8s-master 当中的ip和账户密码，该处填写的是与大脑服务连接的hos个人版机器ip

  - 修改Orin-1 当中的ip和账户密码，该处填写的是大脑机器本身的ip


  ```ini
  # hosts.ini
  [all]
  k8s-master ansible_host=192.168.8.193 ansible_user=root ansible_ssh_pass="supcon" ansible_become=true ansible_become_method=sudo ansible_become_pass="supcon"
  orin-1 ansible_host=192.168.8.161 ansible_user=supconhos ansible_ssh_pass="supcon" ansible_become=true ansible_become_method=sudo ansible_become_pass="supcon"
  
  [cloud]
  k8s-master
  
  [storage]
  k8s-master
  
  [orin]
  orin-1
  
  [k8s:children]
  cloud
  storage
  
  # 全局变量
  [all:vars]
  ansible_python_interpreter=/usr/bin/python3
  
  ```

- 安装**大脑服务**

  ```bash
  ansible-playbook -i orin.ini per-orin.yaml
  ```



## 4. hos集群版服务安装步骤

### 4.1 安装hos 集群版服务 

> **使用场景：当目标机器尚未安装过 HOS 时，可用于在全新环境中部署企业版版 HOS。** 

- 解压 **企业版** 压缩包

  ```bash
  tar -zxvf cluster-install-all-20251112-v0.0.3.tar.gz
  cd cluster_install
  ```

- 配置需要安装hos服务机器的账户密码 `nano hosts.ini`

  ```ini
  # hosts.ini
  [all] # 将集群所有机器的ip全部配置在这里，所有机器必须开放root账户
  k8s-master ansible_host=192.168.8.104 ansible_user=root ansible_ssh_pass="supcon@2025"
  k8s-node   ansible_host=192.168.8.114 ansible_user=root ansible_ssh_pass="supcon@2025"
  
  # 角色组定义（用于匹配 playbook 中的 hosts）
  [master] # 指定 master 节点
  k8s-master
  
  [node] # 指定 node 节点
  k8s-node
  
  [gpu] # 指定gpu节点，该配置会将指定机器的 gpu资源 统一管理，然后提供给集群进行调度
  k8s-node
  
  [storage] # 指定存储节点， 该节点会将指定机器的 存储磁盘资源 统一管理，对集群提供分布式存储能力
  k8s-node
  
  # 总组合并组
  [k8s:children] # 保持不动即可，内部调度使用，无需关心！
  master
  node
  storage
  gpu
  
  # 全局变量
  [all:vars]
  ansible_python_interpreter=/usr/bin/python3
  
  ```

- 执行安装集群版服务

  ```bash
  ansible-playbook -i hosts.ini main.yaml
  ```

- 该步骤安装时间需要大约30min左右，请耐心等待~

- 如有异常，请查看 安装日志或者联系 hos 管理人员

  ```bash
   tail -200f logs/ansible.log
  ```

### 4.2 更新hos 集群版服务 

> **使用场景：当目标机器已经安装过 HOS 时，可用于更新企业版版 HOS。** 

- 解压 **集群版** 压缩包

  ```bash
  tar -zxvf cluster-install-hos-20251112-v0.0.3.tar.gz
  cd cluster_install
  ```

- 配置需要安装hos服务机器的账户密码 `nano hosts.ini`

  ```ini
  # hosts.ini
  [all] # 将集群所有机器的ip全部配置在这里，所有机器必须开放root账户
  k8s-master ansible_host=192.168.8.104 ansible_user=root ansible_ssh_pass="supcon@2025"
  k8s-node   ansible_host=192.168.8.114 ansible_user=root ansible_ssh_pass="supcon@2025"
  
  # 角色组定义（用于匹配 playbook 中的 hosts）
  [master] # 指定 master 节点
  k8s-master
  
  [node] # 指定 node 节点
  k8s-node
  
  [gpu] # 指定gpu节点，该配置会将指定机器的 gpu资源 统一管理，然后提供给集群进行调度
  k8s-node
  
  [storage] # 指定存储节点， 该节点会将指定机器的 存储磁盘资源 统一管理，对集群提供分布式存储能力
  k8s-node
  
  # 总组合并组
  [k8s:children] # 保持不动即可，内部调度使用，无需关心！
  master
  node
  storage
  gpu
  
  # 全局变量
  [all:vars]
  ansible_python_interpreter=/usr/bin/python3
  
  ```

- 执行更新集群版服务

  ```bash
  ansible-playbook -i hosts.ini update.yaml
  ```

- 该步骤安装时间需要大约30min左右，请耐心等待~

- 如有异常，请查看 安装日志或者联系 hos 管理人员

  ```bash
   tail -200f logs/ansible.log
  ```

### 4.3 hos 企业版大脑服务安装 

> **使用场景：当目标机器已经安装过企业版的 HOS 时，可用于安装或更新企业版 HOS对应的大脑服务**
>
> **注意：大脑服务的安装与更新的步骤一致**

#### 操作步骤

- 解压 **个人版大脑**压缩包

  ```bash
  tar -zxvf cluster-install-arm-20251112-v0.0.3.tar.gz
  cd cluster_install
  ```

- 配置需要安装hos服务的大脑服务器的账户密码 `nano orin.ini`

  - 修改k8s-master 当中的ip和账户密码，该处填写的是与大脑服务连接的hos企业版机器ip，该机器必须使用root账户，否则很多命令无法使用会导致更新失败

  - 修改Orin-1 当中的ip和账户密码，该处填写的是大脑机器本身的ip


  ```ini
# hosts.ini
[all]
k8s-master ansible_host=192.168.8.193 ansible_user=root ansible_ssh_pass="supcon" ansible_become=true ansible_become_method=sudo ansible_become_pass="supcon"
orin-1 ansible_host=192.168.8.161 ansible_user=supconhos ansible_ssh_pass="supcon" ansible_become=true ansible_become_method=sudo ansible_become_pass="supcon"

[cloud]
k8s-master

[storage]
k8s-master

[orin]
orin-1

[k8s:children]
cloud
storage

# 全局变量
[all:vars]
ansible_python_interpreter=/usr/bin/python3

  ```

- 安装**大脑服务**

  ```bash
  ansible-playbook -i orin.ini cluster-orin.yaml
  ```

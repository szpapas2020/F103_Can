def bitwise_not_unsigned_char(value):
    # 对value进行按位取反
    inverted_value = ~value

    # 将取反后的值与0xFF进行&操作，确保结果在0-255范围内
    unsigned_inverted_value = inverted_value & 0xFF

    return unsigned_inverted_value

def check_sum(u_buff, u_buff_len):
    u_sum = 0
    for i in range(u_buff_len):
        u_sum = u_sum + u_buff[i]
    u_sum = bitwise_not_unsigned_char(u_sum) + 1
    return u_sum


def print_hex(num):
    # 使用format函数格式化输出，':02x'表示输出小写16进制，不足两位前面补0
    hex_str = format(num, '02x')
    print(hex_str)

buf = [0xA0, 0x04, 0x00, 0xA0, 0x01]

print_hex(check_sum(buf, len(buf)))
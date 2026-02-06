try:
    import px4_msgs.msg
    print("Success: px4_msgs")
except ImportError:
    print("Failed: px4_msgs")

try:
    import px4.msg
    print("Success: px4")
except ImportError:
    print("Failed: px4")
